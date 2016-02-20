package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSetAutoTarget extends Command {
	
    public DriveSetAutoTarget() {
        requires(Robot.driveTrainSubsystem);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        SmartDashboard.putNumber("Proportional (p) drive constant", Robot.driveTrainSubsystem.p);
    	SmartDashboard.putNumber("Proportional (i) drive constant", Robot.driveTrainSubsystem.i);
    	SmartDashboard.putNumber("Proportional (d) drive constant", Robot.driveTrainSubsystem.d);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrainSubsystem.enablePID(true);
    	Robot.driveTrainSubsystem.p = SmartDashboard.getNumber("Proportional (p) drive constant");
    	Robot.driveTrainSubsystem.i = SmartDashboard.getNumber("Proportional (i) drive constant");
    	Robot.driveTrainSubsystem.d = SmartDashboard.getNumber("Proportional (d) drive constant");
    	Robot.driveTrainSubsystem.getPIDController().setPID(Robot.driveTrainSubsystem.p, Robot.driveTrainSubsystem.i, Robot.driveTrainSubsystem.d);
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
    	cancel();
    	end();
    }
}
