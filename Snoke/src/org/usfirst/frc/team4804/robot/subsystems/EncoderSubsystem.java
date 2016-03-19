
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
	public final double DEGREES_PER_PULSE = 1.0/5.92; //360.0 / 497.0; //7 encoder pulses per 1 encoder rev, gearbox reduction is 71:1 ratio; 7*71=497
	public final double TRIGGER_TOLERANCE = 0.05;
	public final double POSITION_TOLERANCE = 5.0;
	public final double POSITION_RANGE_DEG = 137.0;
	public final double POSITION_MAX_DEG = 90.0;
	public final double POSITION_MIN_DEG = -42.0; //degrees below horizontal; need to measure this value
	public final double PULSES_PER_DEGREE = 5.92;
	public final double SPEED_TOLERANCE = 0.03;
	public static double SPEED_MAX = 0.4;
	
	double targetPositionDeg = -45.0; //degrees
	boolean calibrated = false;
	public static boolean auto = false;
	
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
		p = 7;
		i = 0;
		d = 750;
		Robot.cannonEncoderMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		Robot.cannonEncoderMotor.changeControlMode(CANTalon.TalonControlMode.Position);
		setPID(p, i, d);
		SmartDashboard.putNumber("Enc Target angle", targetPositionDeg);
		Robot.cannonEncoderMotor.configMaxOutputVoltage(9.0);
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
		return Robot.cannonEncoderMotor.getEncPosition();
	}
    
    public void setMotorPositionPulses(double pulses) {
    	Robot.cannonEncoderMotor.setPosition((int)pulses);
    }
    
	public double getTargetPositionDeg() {
		if(Robot.vision.distanceFeet != 0) {
			targetPositionDeg = Robot.vision.launchAngle( Robot.vision.distanceFeet * Math.cos(toDeg(getMotorPositionPulses())) ); //horizontal distance
		} else {
			targetPositionDeg = 38.0;
		}
		SmartDashboard.putNumber("Enc Target angle", targetPositionDeg);
		//targetPositionDeg = SmartDashboard.getNumber("Enc Target angle");
		//SmartDashboard.putNumber("Enc Target angle", targetPositionDeg);
		return targetPositionDeg;
	}
    
    public void setPID(double p, double i, double d) {
    	Robot.cannonEncoderMotor.setPID(p, i, d);
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
    	if (auto) {
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
    	setMotorSpeed(xbox.getLeftStickYAxis());
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
    
    //////////////------------------------------------------------------------------------------------------------------------/*
	
    /**
     * Moves the encoder based on camera target position
     */
	/*public void moveTowardTargetPosition() {
    	double currentSpeed = 1;
    	double finalSpeed = 0;
    	double posError = 10;
    	double incGain = 0.1;
    	SmartDashboard.putString("Encoder Command", "Lock Speed Mode");

    	currentSpeed = getMotorSpeed();
    	posError = getTargetPositionDeg() - getMotorPosition();
        if (posError != 0.0) {
	    	currentSpeed = getMotorSpeed();
	    	posError = getTargetPositionDeg() - getMotorPosition();
	    	SmartDashboard.putNumber("Position Error", posError);
	    		    	
	    	if(posError >= POSITION_TOLERANCE) {
	    		// increase speed toward floor if current position is lower than target (too close to the robot)
	    		SmartDashboard.putString("Lock Speed Command", "Incrementing Speed");
	    		finalSpeed = (currentSpeed + incGain*Math.abs(posError));
	    		finalSpeed = (finalSpeed * Math.abs(posError/SPEED_MAX));
	    	}
  
	    	if(posError <= -POSITION_TOLERANCE) {
	    		// increase speed toward robot if current position is greater than target (too close to the floor)
	    		SmartDashboard.putString("Lock Speed Command", "Decrementing Speed");
	    		finalSpeed = (currentSpeed - incGain*Math.abs(posError));
	    		finalSpeed = (finalSpeed * Math.abs(posError/SPEED_MAX));

	    	}
	    	
	    	if(Math.abs(posError) < POSITION_TOLERANCE) {
	    		// do nothing if the current position is within reasonable bounds
	    		SmartDashboard.putString("Lock Speed Command", "Doing Nothing");
	    		finalSpeed = currentSpeed;
	    	}
	    	
	    	SmartDashboard.putString("Lock Speed Command", "Setting Speed");
	    	if(Math.abs(finalSpeed) > SPEED_MAX) {
	    		setMotorSpeed(Math.signum(finalSpeed)*SPEED_MAX);
	    	}else{
	    		setMotorSpeed(finalSpeed);
	    	}
	    	
	    	//Timer.delay(0.005);
	    	
	    	SmartDashboard.putNumber("Final Speed", finalSpeed);
	    	//SmartDashboard.putNumber("Encoder Position Actual", getMotorPosition());
	    	//SmartDashboard.putNumber("Encoder Position Target", getTargetPosition());
	    	SmartDashboard.putString("Lock Speed Command", "Looping");
    	}
    	SmartDashboard.putString("Lock Speed Command", "idle");
    	return;
    }*/
	
	 //PID loop camera centering methods
    /*@Override
	protected double returnPIDInput() {
		return getMotorPosition() - getTargetPosition();
	}

	@Override
	protected void usePIDOutput(double output) {
		setMotorSpeed(output);
	}
	
	public void enablePID() {
		getPIDController().enable();
	}
	
	public void enablePID(boolean enable) {
		if(enable) {
			getPIDController().enable();
		} else {
			getPIDController().disable();
		}
	}*/
    
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	/*public void resetMotor() {
		/*
		//hit first lim
		setMotor(-0.1);
		while(!Robot.cannonEncoderMotor.isFwdLimitSwitchClosed()) {}
		setMotor(0);
		double angle1 = Robot.cannonEncoderMotor.getEncPosition();
		
		//move to second lim
		setMotor(0.1);
		while(!Robot.cannonEncoderMotor.isRevLimitSwitchClosed()) {}
		setMotor(0);
		
		//measure angle
		double angle2 = Robot.cannonEncoderMotor.getEncPosition();
		
		//move until centered
		setMotor(-0.1);
		while(Robot.cannonEncoderMotor.get() > (angle2-angle1)/2) {}
		setMotor(0);*/
		
		
		/*//move down until a limit switch is hit
		setMotorSpeed(-0.2);
		while(!Robot.cannonEncoderMotor.isFwdLimitSwitchClosed() && !Robot.cannonEncoderMotor.isRevLimitSwitchClosed()) {} //neither is pressed
		setMotorSpeed(0);
		//reset encoder value
		Robot.cannonEncoderMotor.setEncPosition(0);
		targetPositionDeg = Robot.cannonEncoderMotor.getEncPosition();
		calibrated = true;
	}*/
	
	
   //PID methods
	/*@Override
	protected double returnPIDInput() {
		double distance = Robot.vision.distanceFeet;
		double distanceX = distance * Math.cos(Robot.cannonEncoderMotor.get()*DEGREES_PER_PULSE - offsetDeg);
		targetPositionDeg = Robot.vision.launchAngle(distanceX) + offsetDeg;
		
		SmartDashboard.putNumber("targetPositionDeg", targetPositionDeg);
		SmartDashboard.putNumber("cannonEncoderMotor.get()", Robot.cannonEncoderMotor.get());
		SmartDashboard.putNumber("cannonEncoderMotor.get() * deg/pulse", Robot.cannonEncoderMotor.get()*DEGREES_PER_PULSE);
		
		//return the position error
		return targetPositionDeg - Robot.cannonEncoderMotor.get()*DEGREES_PER_PULSE;
	}
	
	@Override
	protected void usePIDOutput(double output) {
		setMotor(output);
	}
	
	public void enablePID() {
		getPIDController().enable();
	}
	
	public void enablePID(boolean enable) {
		if(enable) {
			getPIDController().enable();
		} else {
			getPIDController().disable();
		}
	}*/
    
	
	
  //Position calculation/movement/math methods
   //basic move commands
    /*public void moveManual(XboxController xbox){
    	//double speed = -xbox.getLeftStickYAxis()*SPEED_MAX;
    	//setMotor(speed);
    	targetPositionWithTriggers(xbox);
    	move2();
    }
    
    public void moveAuto(double distance){ //feed "distance" from an analog input
    	positionTarget = launchAngle(distance);
    	move();
    }
    
   //use triggers to set target position, automatically correct speed (see last year's code for template)
    public void targetPositionWithTriggers(XboxController xbox){
       //read encoder position
    	double currentPosition = encoder.getRaw();
    	
       //read xbox controller
    	int dpad = xbox.getDPad();
    	double rTrigger = xbox.getRightTriggerAxis();
    	double lTrigger = xbox.getLeftTriggerAxis();
    	
       //calculate target position             Tommy: Let's not set the position target equal to any variant of its current position
    	switch(dpad){
    	
    	case 0:
    		//dpad up
    		if(currentPosition<=POSITION_MAX) //positionTarget = Math.floor(currentPosition/10.0)+10.0;
    			positionTarget += 10;
    		break;
    		
    	case 4:
    		//dpad down
    		if(currentPosition>=10.0) //positionTarget = Math.ceil(currentPosition/10.0)-10.0;
    			positionTarget -= 10;
    		break;
    	
    	/*default:
    		//dpad not pressed, calculate from triggers
	    	if(rTrigger >= TRIGGER_TOLERANCE && positionTarget <= POSITION_MAX){
	    		positionTarget = currentPosition + rTrigger;
	    	}else if(lTrigger >= TRIGGER_TOLERANCE && positionTarget >= 0){
	    		positionTarget = currentPosition - lTrigger;
	    	}else{
	    		//positionTarget = currentPosition;
	    	}
	    	break;*//*
    	}
    	
    	if (positionTarget > POSITION_MAX) positionTarget = POSITION_MAX;
    }
    
   //move and regulate position
    public void move() {
    	double currentPosition = encoder.getRaw();
    	double currentSpeed = encoder.getRate();
    	
       //check position & calculate speed
    	double positionError = currentPosition - positionTarget;
    	double finalSpeed = 0;
    	
    	if(positionError >= POSITION_TOLERANCE) {
    		// increase speed toward floor if current position is too high up
    		SmartDashboard.putString("Lock Speed Command", "Accelerating toward floor");
    		finalSpeed = currentSpeed + Math.abs(positionError);
    		finalSpeed = finalSpeed * Math.abs(positionError/SPEED_MAX);
    	}
    	if(positionError <= -POSITION_TOLERANCE) {
    		// increase speed away from floor if current position is too low
    		SmartDashboard.putString("Lock Speed Command", "Accelerating away from floor");
    		finalSpeed = currentSpeed - Math.abs(positionError);
    		finalSpeed = finalSpeed * Math.abs(positionError/SPEED_MAX);
    	}
    	if(Math.abs(positionError) < POSITION_TOLERANCE) {
    		// do nothing if the current position is within reasonable bounds of target
    		finalSpeed = currentSpeed;
    		if (finalSpeed >= 0) SmartDashboard.putString("Lock Speed Command", "Moving constantly toward floor");
    		if (finalSpeed < 0) SmartDashboard.putString("Lock Speed Command", "Moving constantly away from floor");
    	}
    	
       //set final speed
    	if(Math.abs(finalSpeed) > SPEED_MAX) {
    		//make sure speed isn't out of bounds
    		setMotor(Math.signum(finalSpeed)*SPEED_MAX);
    	}else{
    		setMotor(finalSpeed);
    	}
    	
    	Timer.delay(0.010);	//test different values
    }
    
    public void move2(){
    	double currentPosition = encoder.getRaw();
    	double currentSpeed = encoder.getRate();
    	
    	double positionError = currentPosition - positionTarget;
    	double speed;
    	if (Math.abs(positionError) >= 200){ //sets the speed to travel faster toward the target position when further away
    		speed = SPEED_MAX;
    	}else if (Math.abs(positionError) >= 100){
    		speed = SPEED_MAX*.6;
    	}else if (Math.abs(positionError) >= 50){
    		speed = SPEED_MAX*.3;
    	}else{
    		speed = SPEED_MAX*.1;
    	}
    	SmartDashboard.putNumber("Position Error", positionError);
    	SmartDashboard.putNumber("Speed: ", speed);
    	
    	if (currentPosition > positionTarget && Math.abs(positionError) > POSITION_TOLERANCE){
    		setMotor(speed);
    		SmartDashboard.putString("Motor set: ", "negative");
    	}else if (currentPosition < positionTarget && Math.abs(positionError) > POSITION_TOLERANCE){
    		setMotor(-speed);
    		SmartDashboard.putString("Motor set: ", "positive");
    	}else{ //will only run this if the current position = the target or the error < the tolerance
    		setMotor(0);
    		SmartDashboard.putString("Motor set: ", "nothing");
    	}
    	
    	SmartDashboard.putNumber("CURRENT POSITION", currentPosition);
    	SmartDashboard.putNumber("TARGET POSITION",  positionTarget);
    	
    	Timer.delay(0.010);
    }*/
}


