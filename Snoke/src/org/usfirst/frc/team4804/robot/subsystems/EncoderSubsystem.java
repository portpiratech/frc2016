
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonEncoderMove;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class EncoderSubsystem extends Subsystem {
    
   //objects
	/*private Encoder encoder;
	private DigitalInput inputA = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_A); //The inputs on the RoboRIO DIO
	private DigitalInput inputB = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_B);*/
	
   //constants
	public final double DEGREES_PER_PULSE = 360.0 / (4.0*497.0); //1.0/5.52 //7 encoder pulses per 1 encoder rev, gearbox reduction is 71:1 ratio; 7*71=497
	public final double TRIGGER_TOLERANCE = 0.05;
	public final double POSITION_TOLERANCE = 5.0;
	public final double POSITION_RANGE_DEG = 137.0;
	public final double POSITION_MAX_DEG = 101.0;
	public final double POSITION_MIN_DEG = -38.0; //degrees below horizontal; need to measure this value
	public final double PULSES_PER_DEGREE = 5.52;
	public final double SPEED_TOLERANCE = 0.03;
	public double SPEED_MAX = 0.4;
	
	public double targetPositionDeg = -40; //degrees
	public boolean encPID = false;
	public boolean manualTarget = true;
	
	public double p, i, d;
	
    // Constructor
	public EncoderSubsystem() {
		/*super(0.6, 0.0, 0.3);	//initial PID constants
		p = getPIDController().getP();
		i = getPIDController().getI();
		d = getPIDController().getD();
		getPIDController().setContinuous(false);
		getPIDController().setAbsoluteTolerance(0.05);
		
		getPIDController().disable();*/
		
		super();
		
		//initialize the CANTalon PID stuff
		p = 2;
		i = 0.01;
		d = 200;
		Robot.cannonEncoderMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.Position);
		setPID(p, i, d);
		Robot.cannonEncoderMotor.configMaxOutputVoltage(12.0);
		Robot.cannonEncoderMotor.setAllowableClosedLoopErr(2);
		Robot.cannonEncoderMotor.enableBrakeMode(true);
		
		SmartDashboard.putNumber("Enc Target angle", targetPositionDeg);
		SmartDashboard.putBoolean("Enc manualTarget", manualTarget);
		SmartDashboard.putNumber("Enc const-Proportional (p)", p);
    	SmartDashboard.putNumber("Enc const-Integral (i)", i);
    	SmartDashboard.putNumber("Enc const-Derivative (d)", d);
	}
	
	// Default command
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonEncoderMove());
    }
    
    //METHODS
	
    //POSITION/PID
    public double toDeg(double pulses) {
    	return pulses*DEGREES_PER_PULSE;
    }
    
    public double toPulses(double deg) {
    	return deg/DEGREES_PER_PULSE;
    }
    
    public double getMotorPositionPulses() {
		SmartDashboard.putNumber("EncPosition", Robot.cannonEncoderMotor.getEncPosition());
		return (double)Robot.cannonEncoderMotor.getEncPosition();
	}
    
    public void setMotorPositionPulses(double pulses) {
    	Robot.cannonEncoderMotor.setPosition((int)pulses);
    }
    
	public double getTargetPositionDeg() {
		manualTarget = SmartDashboard.getBoolean("Enc manualTarget");
		if(manualTarget) {
			targetPositionDeg = SmartDashboard.getNumber("Enc Target angle");
		}else{
			//auto target
			if(Robot.vision.distanceFeet != 0 && Robot.driveTrainSubsystem.centered) {
				targetPositionDeg = Robot.vision.launchAngle( Robot.vision.distanceFeet * Math.cos(toDeg(getMotorPositionPulses())*Math.PI/180) ); //horizontal distance
			} else {
				targetPositionDeg = 38.0;
			}
			
			/*if(targetPositionDeg>50.0) {
				targetPositionDeg = 50.0;
			}
			if(targetPositionDeg<25.0) {
				targetPositionDeg = 25.0;
			}*/
			
			SmartDashboard.putNumber("Enc Target angle", targetPositionDeg);
			//targetPositionDeg = SmartDashboard.getNumber("Enc Target angle");
			//SmartDashboard.putNumber("Enc Target angle", targetPositionDeg);
		}
		return targetPositionDeg;
	}
	
	public void setTargetPositionDeg(double degrees) {
		Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.Position);
		targetPositionDeg = degrees;
		SmartDashboard.putNumber("Enc Target angle", targetPositionDeg);
	}
	
	/**
	 * Set the mode of the encoder subsystem
	 * @param PID Enables or disables the encoder PID (enabled = position locking; disabled = manual joystick control)
	 * @param manual Enables or disables manual target angle fetching while PID is enabled. (enabled = set target w/ code or SmartDashboard; disabled = auto-angle [vision] search, not recommended)
	 */
	public void setEncMode(boolean PID, boolean manual) {
		if(PID) {
			encPID = true;
			Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.Position);
		} else {
			encPID = false;
			Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		}
		manualTarget = manual;
	}
    
    public void setPID(double pConst, double iConst, double dConst) {
    	Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.Position);
    	this.p = pConst;
    	this.i = iConst;
    	this.d = dConst;
    	Robot.cannonEncoderMotor.setPID(this.p, this.i, this.d);
    }
    
    public void updatePID() {
    	double pConst = SmartDashboard.getNumber("Enc const-Proportional (p)");
    	double iConst = SmartDashboard.getNumber("Enc const-Integral (i)");
    	double dConst = SmartDashboard.getNumber("Enc const-Derivative (d)");
    	setPID(pConst, iConst, dConst);
    }
    
    public void move(XboxController xbox) {
    	SmartDashboard.putNumber("EncPosition", Robot.cannonEncoderMotor.getEncPosition());
    	SmartDashboard.putNumber("EncVelocity", Robot.cannonEncoderMotor.getEncVelocity());
    	
       //limit switch check
    	//check if encoder is hitting bottom (being reset)
    	if (Robot.cannonEncoderMotor.isRevLimitSwitchClosed()) {
    		SmartDashboard.putString("Is Rev Lim Closed?", "Yes");
    		setMotorPositionPulses(toPulses(POSITION_MIN_DEG));
    		Timer.delay(0.03);
    	}else{
    		SmartDashboard.putString("Is Rev Lim Closed?", "No");
    	}
    	//check if encoder is hitting top
    	if (Robot.cannonEncoderMotor.isFwdLimitSwitchClosed()) {
    		SmartDashboard.putString("Is Fwd Lim Closed?", "Yes");
    		setMotorPositionPulses(toPulses(POSITION_MAX_DEG));
    		Timer.delay(0.03);
    	}else{
    		SmartDashboard.putString("Is Fwd Lim Closed?", "No");
    	}
    	
       //move encoder
    	if (encPID) {
    		setPID(p, i, d);
    		//moveTowardTargetPosition();
    		Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.Position);
    		Robot.cannonEncoderMotor.set(getTargetPositionDeg()*PULSES_PER_DEGREE);
    	} else {
    		setPID(0, 0, 0);
    		Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    		moveXbox(xbox);
    	}
    }
    
    //SPEED/MANUAL
    /**
     * Moves the encoder based on xbox left stick Y axis value
     * @param xbox Xbox controller object
     */
    public void moveXbox(XboxController xbox) {
    	setMotorSpeed(xbox.getRightStickYAxis());
    }
    
    /**
     * Reads encoder velocity
     * @return Current speed of encoder
     */
    public double getMotorSpeed() {
		SmartDashboard.putNumber("EncVelocity", Robot.cannonEncoderMotor.getEncVelocity());
		return Robot.cannonEncoderMotor.getEncVelocity();
	}
	
	/**
	 * Sets encoder to a given speed (scaled to SPEED_MAX in subsystem)
	 * @param speed Value between [-1,1] to control encoder
	 */
	private void setMotorSpeed(double speed) {
		//check if speed is too low to do anything
		if (Math.abs(speed*SPEED_MAX) < SPEED_TOLERANCE) {
			speed = 0;
		}
		//set motor
		Robot.cannonEncoderMotor.set(-speed*SPEED_MAX);
		//targetPositionDeg = Robot.cannonEncoderMotor.getEncPosition();
	}
}


