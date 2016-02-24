
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonSwivelMove;

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
	public static final double SPEED_TOLERANCE = 0.1;
	
	static double angularVelocityLR;
	static double angularVelocityRL;
	
	boolean calibrated = false;
	
  //Initialization:
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonSwivelMove());
    }
    
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
   
   //set motor speed
    public void setMotor(double speed){
    	if (Math.abs(speed) < SPEED_TOLERANCE) speed = 0;
    	Robot.cannonSwivelMotor.set(speed);
    }
    
    public void move(XboxController xbox) {
    	setMotor(xbox.getRightStickXAxis());
    }
    
   //turn motor using xbox input
    public void turn(XboxController xbox){
    	setMotor(xbox.getRightStickXAxis()*speed);
    }
    
   //check if limit switch is pressed
    /**
     * Check if a limit switch wired to the swivel CANTalon is pressed.
     * @param side String: "fwd" or "rev"
     * @return true if pressed, false if not pressed
     */
    public boolean isLimitSwitchPressed(String side) {
    	switch(side) {
    	
    	case "fwd":
    		SmartDashboard.putBoolean("Swivel Right Limit Switch", Robot.cannonSwivelMotor.isFwdLimitSwitchClosed());
    		return Robot.cannonSwivelMotor.isFwdLimitSwitchClosed();
    	case "rev":
    		SmartDashboard.putBoolean("Swivel Left Limit Switch", Robot.cannonSwivelMotor.isRevLimitSwitchClosed());
    		return Robot.cannonSwivelMotor.isRevLimitSwitchClosed();
    	default: return false;
    	}
    }
    
    
   //calibrate motor--experimentally determine angular velocity with given angular displacement, measuring elapsed time
    public void calibrateSpeed() {
    	//rotate left until left limit switch is hit to reset
    	setMotor(-speed);
    	while(!isLimitSwitchPressed("rev")) {}
    	
    	//stop motor
    	setMotor(0);
    	
    	//record time and rotate right until right limit switch is hit
    	double time0 = System.currentTimeMillis();
    	setMotor(speed);
    	while(!isLimitSwitchPressed("fwd")) {}
    	
    	//record time and rotate left until left limit switch is hit
    	double time1 = System.currentTimeMillis();
    	setMotor(-speed);
    	while(!isLimitSwitchPressed("rev")) {}
    	
    	double time2 = System.currentTimeMillis();
    	
    	double elapsedTimeLR = time1 - time0; //don't feed the robots past midnight
    	double elapsedTimeRL = time2 - time1;
    	angularVelocityLR = ANGULAR_DISPLACEMENT / elapsedTimeLR;
    	angularVelocityRL = ANGULAR_DISPLACEMENT / elapsedTimeRL;
    	
    	calibrated = true;
    }
    
    public void move(String direction) {
    	switch(direction) {
    	case "LR":
    		setMotor(angularVelocityLR);
    		break;
    	case "RL":
    		setMotor(angularVelocityRL);
    		break;
    	}
    }
    
}


