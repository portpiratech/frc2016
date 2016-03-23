
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Load extends Command {
	
    public Load() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cannonSubsystem);
        requires(Robot.pusherSubsystem);
        requires(Robot.visionSubsystem);
        requires(Robot.encoderSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.pusherSubsystem.positionReverse(); //retract pusher
    	Robot.cannonSubsystem.motorLoad(); //load cannon motors
    	Robot.encoderSubsystem.setEncMode(false, true); //disable encoder locking so manual control can be used
    	Robot.visionSubsystem.disableProcessing(); //disable processing to free up resources during loading
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
