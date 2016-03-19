
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CannonPusherReverse extends Command {

	boolean finished = false;
	
    public CannonPusherReverse() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.pusherSubsystem);
    	setInterruptible(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	finished = false;
    	Robot.pusherSubsystem.positionReverse();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.pusherSubsystem.positionReverse();
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
    	finished = true;
    	isFinished();
    }
}
