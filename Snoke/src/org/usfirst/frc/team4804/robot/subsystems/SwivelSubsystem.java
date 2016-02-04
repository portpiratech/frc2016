
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonSwivelRotate;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Horizontal movement system for cannon. Uses windshield motor to rotate.
 */
public class SwivelSubsystem extends Subsystem {
    
	//Currently using: WINDSHIELD MOTOR
	
  //Constants:
	public static final double SPEED = .1;
	public static final double ANGULAR_DISPLACEMENT = 10; //angle between limit switches, degrees
	
  //Initialization:
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonSwivelRotate());
    }
    
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
   
   //set motor speed
    public void setMotor(double speed){
    	Robot.cannonSwivelMotor.set(speed);
    }
    
   //turn motor using xbox input
    public void turn(XboxController xbox){
    	setMotor(xbox.getRightStickXAxis()*SPEED);
    }
    
   //check if limit switch is pressed
    public boolean limitSwitchPressed(String side) {
    	switch(side) {
    	
    	case "left":
    		SmartDashboard.putBoolean("Swivel Right Limit Switch", Robot.limitLeft.get());
    		return !Robot.limitLeft.get();
    	case "right":
    		SmartDashboard.putBoolean("Swivel Left Limit Switch", Robot.limitRight.get());
    		return !Robot.limitRight.get();
    	default: return false;
    	}
    }
    
    
   //calibrate motor--experimentally determine angular velocity with given angular displacement, measuring elapsed time
    public void calculateSpeed(){
    	double time0 = System.currentTimeMillis();
    	setMotor(SPEED);
    	while(!limitSwitchPressed("right")) {}
    	
    	double time1 = System.currentTimeMillis();
    	double elapsedTime = time1 - time0; //don't feed the robots past midnight
    	double angularVelocity = ANGULAR_DISPLACEMENT / elapsedTime;
    }
    
    
    
}


