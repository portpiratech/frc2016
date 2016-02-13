
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
    
	//Currently using: WINDSHIELD WIPER MOTOR (No encoding)
	
  //Constants:
	public static double speed = .1;
	public static final double ANGULAR_DISPLACEMENT = 10; //angle between limit switches, degrees. Needs to be measured
	
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
    	setMotor(xbox.getRightStickXAxis()*speed);
    }
    
   //check if limit switch is pressed
    /**
     * Check if a limit switch is pressed.
     * @param side String: "right" or "left"
     * @return true if not pressed, false if pressed
     */
    public boolean isLimitSwitchPressed(String side) {
    	switch(side) {
    	
    	case "left":
    		SmartDashboard.putBoolean("Swivel Right Limit Switch", Robot.limitLeft.get());
    		return Robot.limitLeft.get();
    	case "right":
    		SmartDashboard.putBoolean("Swivel Left Limit Switch", Robot.limitRight.get());
    		return Robot.limitRight.get();
    	default: return false;
    	}
    }
    
    
   //calibrate motor--experimentally determine angular velocity with given angular displacement, measuring elapsed time
    public void calculateSpeed(){
    	//rotate left until left limit switch is hit to reset
    	setMotor(-speed);
    	while(!isLimitSwitchPressed("left")) {}
    	
    	//stop motor
    	setMotor(0);
    	
    	//record time and rotate right until right limit switch is hit
    	double time0 = System.currentTimeMillis();
    	setMotor(speed);
    	while(!isLimitSwitchPressed("right")) {}
    	
    	//record time and rotate left until left limit switch is hit
    	double time1 = System.currentTimeMillis();
    	setMotor(-speed);
    	while(!isLimitSwitchPressed("left")) {}
    	
    	double time2 = System.currentTimeMillis();
    	
    	double elapsedTimeLR = time1 - time0; //don't feed the robots past midnight
    	double elapsedTimeRL = time2 - time1;
    	double angularVelocityLR = ANGULAR_DISPLACEMENT / elapsedTimeLR;
    	double angularVelocityRL = ANGULAR_DISPLACEMENT / elapsedTimeRL;
    }
    
    
    
}


