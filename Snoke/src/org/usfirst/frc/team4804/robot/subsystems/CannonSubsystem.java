
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.ExampleCommand;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CannonSubsystem extends Subsystem {
    
	// Wheel launcher variables
	public static double motorSpeed = 0;
	public static final double LOAD_SPEED = -0.1;
	public static final double LAUNCH_SPEED = 0.2;
	
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
    	setMotors(LOAD_SPEED);
    }
    
    public void motorLaunch() {
    	setMotors(LAUNCH_SPEED);
    }
    
    public void setMotors(double speed) {
    	Robot.cannonShooterMotors.set(speed); // check if this is the right method
    }
    
    public void setCannonPosition(double position) {
    	Robot.cannonTiltMotor.set(position); // check if this is the right method
    }
}


