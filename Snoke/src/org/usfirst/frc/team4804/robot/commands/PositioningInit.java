
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Get the robot ready for auto-positioning/lining up with target
 */
public class PositioningInit extends Command {

	boolean finished = true;
	
    public PositioningInit() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.encoderSubsystem);
        //requires(Robot.visionSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.encoderSubsystem.setEncMode(true, true); //enable encoder position locking and manual target setting
    	Robot.encoderSubsystem.setTargetPositionDeg(40.0);
    	Robot.encoderSubsystem.targetPositionDeg = SmartDashboard.getNumber("Enc Target angle");
    	//Robot.visionSubsystem.enableProcessing(); //enable vision processing
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
