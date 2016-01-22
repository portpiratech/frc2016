
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CannonSubsystem extends Subsystem {
    
	// Put important variables and objects for this subsystem here.
	
	// Wheel launcher variables
	public static double motorSpeed = 0;
	public static final double LOAD_SPEED_L = -0.3;
	public static final double LOAD_SPEED_R = -1*LOAD_SPEED_L;
	public static final double LAUNCH_SPEED_L = 1.0;
	public static final double LAUNCH_SPEED_R = -1*LAUNCH_SPEED_L;
	
	// Cannon positioning variables
	public static double tiltAngle = 0;
	public static double tiltSpeed = 0;
	public static final double LOADING_POSITION = 0;
	public static final double LAUNCHING_POSITION = 0;
	
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    
    // Loading/Launching the ball
    public void motorLoad() {
    	setMotor("L", LOAD_SPEED_L);
    	setMotor("R", LOAD_SPEED_R);
    }
    
    public void motorLaunch() {
    	setMotor("L", LAUNCH_SPEED_L);
    	setMotor("R", LAUNCH_SPEED_R);
    }
    
    public void motorStop() {
    	setMotor("L", 0);
    	setMotor("R", 0);
    }
    
    public void setMotor(String side, double speed) {
    	// side "L" = left
    	// side "R" = right1
    		
    	switch(side) {
    		case "L": Robot.cannonLauncherMotorLeft.set(speed); // check if this is the right method
    		case "R": Robot.cannonLauncherMotorRight.set(speed);
    	}
    }
    
    
    // Positioning
    public void setCannonPosition(double position) {
//    	Robot.cannonTiltMotor.set(position); // check if this is the right method
    }
    
    
    // Set the default command for a subsystem here.
    public void initDefaultCommand() {
        // setDefaultCommand(new ExampleCommand());
    }
}


