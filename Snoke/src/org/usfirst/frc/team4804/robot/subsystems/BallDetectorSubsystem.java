
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.BallDetector;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class BallDetectorSubsystem extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new BallDetector());
    }
    
    public void checkIfDetected() {
    	boolean detected;
    	if (Robot.ballDetectorLim.get()) {
    		detected = true;
    	} else {
    		detected = false;
    	}
    	SmartDashboard.putBoolean("Ball Loaded?", detected);
    	//return detected;
    }
}


