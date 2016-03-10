
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CannonPusherMove extends Command {

	boolean finished = false;
	
    public CannonPusherMove() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.pusherSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	finished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.pusherSubsystem.positionJoystick(Robot.oi.driverController);
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
