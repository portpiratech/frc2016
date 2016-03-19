
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PositioningInit extends Command {

	boolean finished = true;
	
    public PositioningInit() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.encoderSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	EncoderSubsystem.auto = true;
    	EncoderSubsystem.manualTarget = true;
    	SmartDashboard.putNumber("Enc Target angle", 45.0);
    	Robot.encoderSubsystem.targetPositionDeg = SmartDashboard.getNumber("Enc Target angle");
    	VisionSubsystem.visionProcessing = true;
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
