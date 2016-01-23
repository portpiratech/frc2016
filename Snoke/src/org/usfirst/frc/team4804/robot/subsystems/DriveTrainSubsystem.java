
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.DriveCommand;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrainSubsystem extends Subsystem {
    
	public static final double DRIVE_SPEED = 0.15;	// maximum drive speed; scales other speeds to this
	public static final double DPAD_MULT = 0.5;		// multiplier for dpad speed controls.
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveCommand());
    }
    
 // Methods
    
    public void setMotor(String side, double speed) {
    	// side "L" = left
    	// side "R" = right
    	
    	if (Math.abs(speed) < .05) speed = 0; //controller axis always returns insignificant values, fixes creeping
    		
    	switch(side) { //CANTalons are hooked up pos-pos && neg-neg
    		case "L":
    			Robot.tankDriveLeft.set(-speed);
    			SmartDashboard.putNumber("Drive train left:", -speed);
    			break;
    		case "R": 
    			Robot.tankDriveRight.set(speed);
    			SmartDashboard.putNumber("Drive train right:", speed);
    			break;
    	}
    }
    
    public void drive(XboxController xbox) {
    	//standardDrive(xbox.getLeftStickYAxis(), xbox.getRightStickYAxis());
    	jonnyDrive(xbox.getLeftStickYAxis(), xbox.getLeftStickXAxis(), xbox.getDPad());
    	//tommyDrive(xbox.getLeftStickXAxis(), xbox.getLeftStickYAxis(), xbox.getDPad());
    }
    
    // uses two joysticks, left stick y-axis and right stick y-axis
    public void standardDrive(double leftY, double rightY) {
    	setMotor("L", leftY*DRIVE_SPEED);
    	setMotor("R", rightY*DRIVE_SPEED);
    }
    
    // uses one joystick, left stick y-axis for magnitude, left stick x-axis for direction
    public void jonnyDrive(double leftY, double leftX, int dpad){ //left stick's y value and left stick's x value
    											//xSpeed and ySpeed range from -1 to 1 based on % of max speed
    	if (dpad == -1){
	    	double leftMotorSpeed = leftY;
	    	double rightMotorSpeed = leftY;
	    	
	    	if (leftY < 0){                   //increments turning
	    		leftMotorSpeed += -leftX;
	        	rightMotorSpeed += leftX;
	    	}else{
	    		leftMotorSpeed += leftX;
	        	rightMotorSpeed += -leftX;
	    	}
	    	
	    	if (leftMotorSpeed > 1.0)  leftMotorSpeed =  1.0;  //fixes values from being out of bounds
	    	if (leftMotorSpeed < -1.0) leftMotorSpeed = -1.0;
	    	if (rightMotorSpeed > 1.0)  rightMotorSpeed =  1.0;
	    	if (rightMotorSpeed < -1.0) rightMotorSpeed = -1.0;
	    	
	    	setMotor("L", leftMotorSpeed*DRIVE_SPEED);
	    	setMotor("R", rightMotorSpeed*DRIVE_SPEED);
    	}else{
    		switch(dpad){ //default dpad directions (perfectly straight, back, cw, ccw);
    					  //negative is forward specifically here... for some reason...
    		case 0: //forward
    			setMotor("L", -DRIVE_SPEED);
    			setMotor("R", -DRIVE_SPEED);
    			break;
    		case 2: //cw
    			setMotor("L", -DRIVE_SPEED);
    			setMotor("R", DRIVE_SPEED);
    			break;
    		case 4: //backward
    			setMotor("L", DRIVE_SPEED);
    			setMotor("R", DRIVE_SPEED);
    			break;
    		case 6: //ccw
    			setMotor("L", DRIVE_SPEED);
    			setMotor("R", -DRIVE_SPEED);
    			break;
    		}
    	}
    }
    
    // uses one joystick, left stick x- and y-axis
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
	    	leftMotorSpeed *= magnitude*DRIVE_SPEED;
	    	rightMotorSpeed *= magnitude*DRIVE_SPEED;
	    	
	    	SmartDashboard.putNumber("Angle:", angle);
	    	SmartDashboard.putNumber("Magnitude:", magnitude);
	    	SmartDashboard.putNumber("Quadrant:", quadrant);
	    	
	    	setMotor("L", leftMotorSpeed);
	    	setMotor("R", rightMotorSpeed);
	    }else{
			switch(dpad){ //default dpad directions (perfectly straight, back, cw, ccw);
						  //negative is forward specifically here... for some reason...
			case 0: //forward
				setMotor("L", -DRIVE_SPEED*DPAD_MULT);
				setMotor("R", -DRIVE_SPEED*DPAD_MULT);
				break;
			case 2: //cw
				setMotor("L", -DRIVE_SPEED*DPAD_MULT);
				setMotor("R", DRIVE_SPEED*DPAD_MULT);
				break;
			case 4: //backward
				setMotor("L", DRIVE_SPEED*DPAD_MULT);
				setMotor("R", DRIVE_SPEED*DPAD_MULT);
				break;
			case 6: //ccw
				setMotor("L", DRIVE_SPEED*DPAD_MULT);
				setMotor("R", -DRIVE_SPEED*DPAD_MULT);
				break;
			}
		}
    }
    
}


