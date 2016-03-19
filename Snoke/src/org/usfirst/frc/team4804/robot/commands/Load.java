
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Load extends Command {
	
    public Load() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cannonSubsystem);
        requires(Robot.pusherSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.pusherSubsystem.positionReverse();
    	Robot.cannonSubsystem.motorLoad();
    	EncoderSubsystem.auto = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.pusherSubsystem.positionReverse();
    	Robot.cannonSubsystem.motorLoad();
    	EncoderSubsystem.auto = false;
    	
    	// set rumble
    	//Robot.oi.operatorController.setRumble(RumbleType.kLeftRumble, (float)0.5);
    	//Robot.oi.operatorController.setRumble(RumbleType.kRightRumble, (float)0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
