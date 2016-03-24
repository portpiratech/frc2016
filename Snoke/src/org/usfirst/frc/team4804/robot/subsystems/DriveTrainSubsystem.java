package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.RobotModes;
import org.usfirst.frc.team4804.robot.commands.DriveCommand;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for controlling the two drive motors.
 */
public class DriveTrainSubsystem extends PIDSubsystem {
    
	//multipliers
	public static double driveSpeed = 1.0;	// maximum drive speed; scales other speeds to this
	public static double dpadMult = 0.25;		// multiplier for dpad speed controls.
	
	public static final double SPEED_TOLERANCE = 0.1; // can't be too close to 0
	static int driveSetting = 0; //0 is tank, 1 is Jonnydrive, 2 is Tommydrive
    public boolean centered = false;
	
	public double p, i, d;
	
	public DriveTrainSubsystem() {
		super(0.5, 0.0, 0.3);	//initial PID constants
		p = getPIDController().getP();
		i = getPIDController().getI();
		d = getPIDController().getD();
		
		getPIDController().setContinuous(false);
		getPIDController().setAbsoluteTolerance(0.01);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveCommand());
    }
    
 // Methods
    /**
	 * Sets motor speeds.
	 * @param side String, either "L" or "R"
	 * @param speed Speed in range [-1,1]
	 */
    public void setMotor(String side, double speed) {
    	// side "L" = left
    	// side "R" = right
    	
    	if (Math.abs(speed) < SPEED_TOLERANCE) speed = 0; //controller axis always returns insignificant values, fixes creeping
    		
    	switch(side) { //CANTalons are hooked up pos-pos && neg-neg
    		case "L":
    			if(Robot.currentMode == RobotModes.NEW_ROBOT_MODE) Robot.tankDriveLeft.set(speed);
    			if(Robot.currentMode == RobotModes.TEST_ROBOT_MODE) Robot.tankDriveLeftTest.set(speed);
    			SmartDashboard.putNumber("Drive train left:", speed);
    			break;
    		case "R": 
    			if(Robot.currentMode == RobotModes.NEW_ROBOT_MODE) Robot.tankDriveRight.set(-speed);
    			if(Robot.currentMode == RobotModes.TEST_ROBOT_MODE) Robot.tankDriveRightTest.set(-speed);
    			SmartDashboard.putNumber("Drive train right:", -speed);
    			break;
    	}
    }
    
    /**
     * Runs the robot from the xbox controller.
     * @param xbox Xbox controller input.
     */
    public void drive(XboxController xbox) {
    	
    	switch(driveSetting){
    	case 0:
    		tankDrive(xbox.getLeftStickYAxis(), xbox.getRightStickYAxis(), xbox.getDPad());
    		SmartDashboard.putString("Drive Setting", "Tank Drive");
    		break;
    	case 1:
    		jonnyDrive(xbox.getLeftStickYAxis(), xbox.getLeftStickXAxis(), xbox.getDPad(), xbox.getRightStickXAxis());
    		SmartDashboard.putString("Drive Setting", "Jonny Drive");
    		break;
    	case 2:
    		tommyDrive(xbox.getLeftStickXAxis(), xbox.getLeftStickYAxis(), xbox.getDPad());
    		SmartDashboard.putString("Drive Setting", "Tommy Drive");
    		break;
    	}
    }
    
    /**
     * Changes the drive mode. 0=tank, 1=jonny, 2=tommy.
     */
    public void toggleDriveSetting(){ //mapped to A button on driver's controller
    	driveSetting += 2;
    	if (driveSetting > 2){
    		driveSetting = 0;
    	}
    }
    
    /**
     * Changes the drive mode. 0=tank, 1=jonny, 2=tommy.
     */
    public void toggleDriveSetting(String driveMode){ //mapped to A button on driver's controller
    	switch(driveMode) {
    	case "tank":
    		driveSetting = 0;
    		break;
    	case "jonny":
    		driveSetting = 1;
    		break;
    	case "tommy":
    		driveSetting = 2;
    		break;
    	default:
    		toggleDriveSetting();
    		break;
    	}
    }
    
    // uses two joysticks, left stick y-axis and right stick y-axis
    /**
     * Tank-style driving method.
     * @param leftY Left joystick's y-value. Speed in range [-1,1] controlling the left motors
     * @param rightY Right joystick's y-value. Speed in range [-1,1] controlling the right motors
     */
    public void tankDrive(double leftY, double rightY, int dpad) {
    	if(dpad == -1) {
	    	setMotor("L", leftY*driveSpeed);
	    	setMotor("R", rightY*driveSpeed);
    	} else{
			switch(dpad){ //default dpad directions (perfectly straight, back, cw, ccw);
			  //negative is forward specifically here... for some reason...
			case 0: //forward
				setMotor("L", -driveSpeed*dpadMult);
				setMotor("R", -driveSpeed*dpadMult);
				break;
			case 2: //cw
				setMotor("L", -driveSpeed*dpadMult);
				setMotor("R", driveSpeed*dpadMult);
				break;
			case 4: //backward
				setMotor("L", driveSpeed*dpadMult);
				setMotor("R", driveSpeed*dpadMult);
				break;
			case 6: //ccw
				setMotor("L", driveSpeed*dpadMult);
				setMotor("R", -driveSpeed*dpadMult);
				break;
			}
    	}
    }
    
    // uses one joystick, left stick y-axis for magnitude, x-axis for direction. car style
    /**
     * Jonny's custom drive method. 
     * @param leftY Left joystick's y-value (in range [-1,1]).
     * @param leftX Left joystick's x-value (in range [-1,1]).
     * @param dpad Dpad input value 0, 2, 4, or 6. Strict directional driving (forward, turn right, backward, turn left).
     * @param rightX Right joystick's x-value. Speed in range [-1,1] controlling miniscule turning.
     */
    public void jonnyDrive(double leftY, double leftX, int dpad, double rightX){ //left stick's y value and left stick's x value
    											//xSpeed and ySpeed range from -1 to 1 based on % of max speed
    	if (dpad == -1){
	    	double leftMotorSpeed = leftY;
	    	double rightMotorSpeed = leftY;
	    	
	    	if (true){                   //increments motor speeds for turning
	    		leftMotorSpeed += -leftX;
	        	rightMotorSpeed += leftX;
	    	}/*else{
	    		leftMotorSpeed += leftX;
	        	rightMotorSpeed += -leftX;
	    	}*/
	    	
	    	if (leftMotorSpeed > 1.0)  leftMotorSpeed =  1.0;  //fixes values from being out of bounds
	    	if (leftMotorSpeed < -1.0) leftMotorSpeed = -1.0;
	    	if (rightMotorSpeed > 1.0)  rightMotorSpeed =  1.0;
	    	if (rightMotorSpeed < -1.0) rightMotorSpeed = -1.0;
	    	
	    	if (Math.abs(leftMotorSpeed) < SPEED_TOLERANCE && Math.abs(rightMotorSpeed) < SPEED_TOLERANCE){ //use the right stick for miniscule turning
	    		leftMotorSpeed = -rightX * dpadMult;
	    		rightMotorSpeed = rightX * dpadMult;
	    	}
	    	
	    	setMotor("L", leftMotorSpeed*driveSpeed);
	    	setMotor("R", rightMotorSpeed*driveSpeed);
    	}else{
    		switch(dpad){ //default dpad directions (perfectly straight, back, cw, ccw);
    					  //negative is forward specifically here... for some reason...
    		case 0: //forward
    			setMotor("L", -driveSpeed*dpadMult);
    			setMotor("R", -driveSpeed*dpadMult);
    			break;
    		case 2: //cw
    			setMotor("L", -driveSpeed*dpadMult);
    			setMotor("R", driveSpeed*dpadMult);
    			break;
    		case 4: //backward
    			setMotor("L", driveSpeed*dpadMult);
    			setMotor("R", driveSpeed*dpadMult);
    			break;
    		case 6: //ccw
    			setMotor("L", driveSpeed*dpadMult);
    			setMotor("R", -driveSpeed*dpadMult);
    			break;
    		}
    	}
    }
    
    // uses one joystick, left stick x- and y-axis, vector style
    /**
     * Tommy's custom drive method. Vector-style driving (based on angle + magnitude of the left joystick).
     * @param leftX Left joystick's x-value (in range [-1,1]).
     * @param leftY Left joystick's y-value (in range [-1,1]).
     * @param dpad Dpad input value 0, 2, 4, or 6. Strict directional driving (forward, turn right, backward, turn left).
     */
    public void tommyDrive(double leftX, double leftY, int dpad) {
    	
    	leftX *= -1; //corrects input value
    	
    	if(dpad == -1) {
    		//calculate the angle. arctangent returns values between -pi/2 and +pi/2, so correct it.
	    	double angle = Math.atan(leftY/leftX);
	    //	if(leftX>=0 && leftY>=0) angle = angle;		// quadrant I
	    	if(leftX< 0 && leftY>=0) angle += Math.PI;	// quadrant II
	    	if(leftX< 0 && leftY< 0) angle += Math.PI;	// quadrant III
	    	if(leftX>=0 && leftY< 0) angle += 2*Math.PI;	// quadrant IV
	    	
	    	double magnitude = Math.sqrt( Math.pow(leftX, 2) + Math.pow(leftY, 2) );
	    	
	    	double leftMotorSpeed = 0;
	    	double rightMotorSpeed = 0;
	    	int quadrant = 0;
	    	
	    	if(angle>=0 && angle<Math.PI/2) {
	    		leftMotorSpeed = 1;
	    		rightMotorSpeed = -Math.cos(2*angle);
	    		quadrant = 1;
	    	}
	    	if(angle>=Math.PI/2 && angle<Math.PI) {
	    		leftMotorSpeed = -Math.cos(2*angle);
	    		rightMotorSpeed = 1;
	    		quadrant = 2;
	    	}
	    	if(angle>=Math.PI && angle<Math.PI*3/2) {
	    		leftMotorSpeed = -1;
	    		rightMotorSpeed = Math.cos(2*angle);
	    		quadrant = 3;
	    	}
	    	if(angle>=Math.PI*3/2 && angle<=2*Math.PI) {
	    		leftMotorSpeed = Math.cos(2*angle);
	    		rightMotorSpeed = -1;
	    		quadrant = 4;
	    	}
	    	
	    //  scale motor speeds to joystick magnitude, max speed
	    	leftMotorSpeed *= magnitude*driveSpeed;
	    	rightMotorSpeed *= magnitude*driveSpeed;
	    	
	    	SmartDashboard.putNumber("Angle:", angle);
	    	SmartDashboard.putNumber("Magnitude:", magnitude);
	    	SmartDashboard.putNumber("Quadrant:", quadrant);
	    	
	    	setMotor("L", leftMotorSpeed);
	    	setMotor("R", rightMotorSpeed);
	    }else{
			switch(dpad){ //default dpad directions (perfectly straight, back, cw, ccw);
						  //negative is forward specifically here... for some reason...
			case 0: //forward
				setMotor("L", -driveSpeed*dpadMult);
				setMotor("R", -driveSpeed*dpadMult);
				break;
			case 2: //cw
				setMotor("L", -driveSpeed*dpadMult);
				setMotor("R", driveSpeed*dpadMult);
				break;
			case 4: //backward
				setMotor("L", driveSpeed*dpadMult);
				setMotor("R", driveSpeed*dpadMult);
				break;
			case 6: //ccw
				setMotor("L", driveSpeed*dpadMult);
				setMotor("R", -driveSpeed*dpadMult);
				break;
			}
		}
    }
    
    //crude camera centering method
    /*public void autoCenterCamera() {
    	double error = Robot.vision.errorAimingX;
    	double speed = 0;
    	
    	//center at error=0
    	
    	if(Math.abs(error)>0.002) speed = error*driveSpeed*50; //if error<0, then speed<0. if error>0, then speed>0.
    	if(Math.abs(speed)>1) speed = Math.signum(speed);
    	
    	setMotor("L", -speed);
		setMotor("R", speed);
    }*/
    
    //PID loop camera centering methods--use these!
    @Override
	protected double returnPIDInput() {
    	if(Robot.vision.errorAimingX < 0.01) {
    		//check if close enough to centered
    		centered = true;
    	}
		return Robot.vision.errorAimingX;
	}

	@Override
	protected void usePIDOutput(double output) {
		setMotor("L", output);
		setMotor("R", -output);
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
	}
	
	//PID constants
	public void setPID(double p, double i, double d) {
    	Robot.driveTrainSubsystem.p = p;
    	Robot.driveTrainSubsystem.i = i;
    	Robot.driveTrainSubsystem.d = d;
    	Robot.driveTrainSubsystem.getPIDController().setPID(p, i, d);
    }
    
    public void updatePID() {
    	double p = SmartDashboard.getNumber("Drive const-Proportional (p)");
    	double i = SmartDashboard.getNumber("Drive const-Integral (i)");
    	double d = SmartDashboard.getNumber("Drive const-Derivative (d)");
    	setPID(p, i, d);
    }
}


