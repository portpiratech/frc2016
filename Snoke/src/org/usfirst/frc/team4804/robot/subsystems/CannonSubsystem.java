
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CannonSubsystem extends Subsystem {
    
	// Wheel launcher variables
	public static double motorSpeed = 0;
	// public static final double LOAD_SPEED = -0.1;
	// public static final double LAUNCH_SPEED = 0.2;
	
	public static final double LOAD_SPEED_L = -0.1;
	public static final double LOAD_SPEED_R = -0.15;
	public static final double LAUNCH_SPEED_L = 0.2;
	public static final double LAUNCH_SPEED_R = 0.2;
	
	// Cannon positioning variables
	public static double tiltAngle = 0;
	public static double tiltSpeed = 0;
	
	public static final double LOADING_POSITION = 0;
	public static final double LAUNCHING_POSITION = 0;
	
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new ExampleCommand());
    }
    
    public void motorLoad() {
    	setMotor("L", LOAD_SPEED_L);
    	setMotor("R", LOAD_SPEED_R);
    }
    
    public void motorLaunch() {
    	setMotor("L", LAUNCH_SPEED_L);
    	setMotor("R", LAUNCH_SPEED_R);
    }
    
    public void setMotor(String side, double speed) {
    	// side "L" = left
    	// side "R" = right1
    	switch(side) {
    		case "L": Robot.cannonShooterMotorLeft.set(speed); // check if this is the right method
    		case "R": Robot.cannonShooterMotorRight.set(speed);
    	}
    }
    
    public void setCannonPosition(double position) {
    	Robot.cannonTiltMotor.set(position); // check if this is the right method
    }
}


