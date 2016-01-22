
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.DriveCommand;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrainSubsystem extends Subsystem {
    
	public static final double DRIVE_SPEED = 0.5;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveCommand());
    }
    
    public void drive(XboxController xbox) {
    	jonnyDrive(xbox.getLeftStickYAxis(), xbox.getRightStickXAxis());
    }
    
    public void jonnyDrive(double leftY, double rightX){ //left stick's y value and right stick's x value
    											//xSpeed and ySpeed range from -1 to 1 based on % of max speed
    	double leftMotorSpeed = leftY;
    	double rightMotorSpeed = leftY;
    	leftMotorSpeed += rightX;
    	rightMotorSpeed += -rightX;
    	
    	if (leftMotorSpeed > 1.0)  leftMotorSpeed =  1.0;
    	if (leftMotorSpeed < -1.0) leftMotorSpeed = -1.0;
    	if (rightMotorSpeed > 1.0)  rightMotorSpeed =  1.0;
    	if (rightMotorSpeed < -1.0) rightMotorSpeed = -1.0;
    	
    	setMotor("L", leftMotorSpeed*DRIVE_SPEED);
    	setMotor("R", rightMotorSpeed*DRIVE_SPEED);
    }
    
    public void setMotor(String direction, double speed) {
    	// side "L" = left
    	// side "R" = right1
    		
    	switch(direction) {
    		case "L": Robot.tankDriveLeft.set(speed); // check if this is the right method
    		case "R": Robot.tankDriveRight.set(speed);
    	}
    }
    
}


