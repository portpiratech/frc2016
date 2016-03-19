package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TargetingManual extends Command {
	
	boolean drive_ = true;
	boolean encoder_ = true;
	boolean swivel_ = false;
	
    public TargetingManual() {
        if(drive_) requires(Robot.driveTrainSubsystem);
        if(encoder_) requires(Robot.encoderSubsystem);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(drive_) Robot.driveTrainSubsystem.enablePID(false);
    	if(encoder_) EncoderSubsystem.auto = false;
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
    	cancel();
    	end();
    }
}
