
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Launch extends Command {
	
    public Launch() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cannonSubsystem);
        requires(Robot.pusherSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.pusherSubsystem.positionReverse();
    	VisionSubsystem.visionProcessing = true;
    	EncoderSubsystem.auto = true;
    	EncoderSubsystem.manualTarget = true;
    	SmartDashboard.putNumber("Enc Target angle", SmartDashboard.getNumber("Launch Angle (Test)"));
    	Robot.encoderSubsystem.targetPositionDeg = SmartDashboard.getNumber("Enc Target angle");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.cannonSubsystem.motorLaunch();
    	Timer.delay(2);
    	Robot.pusherSubsystem.positionCenter();
    	Timer.delay(3);
    	Robot.pusherSubsystem.positionReverse();
    	VisionSubsystem.visionProcessing = false;
    	EncoderSubsystem.auto = false;
    	// set rumble
    	//Robot.oi.operatorController.setRumble(RumbleType.kLeftRumble, (float)0.5);
    	//Robot.oi.operatorController.setRumble(RumbleType.kRightRumble, (float)0);
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
