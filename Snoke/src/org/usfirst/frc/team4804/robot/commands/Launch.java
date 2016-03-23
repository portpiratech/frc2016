
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Launch the ball (only call after auto-positioning has been completed)
 */
public class Launch extends Command {
	
    public Launch() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cannonSubsystem);
        requires(Robot.pusherSubsystem);
        requires(Robot.encoderSubsystem);
        requires(Robot.visionSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.pusherSubsystem.positionReverse(); //make sure pusher is retracted
    	Robot.visionSubsystem.enableProcessing(); //enable vision processing so angle can be calculated
    	Robot.encoderSubsystem.setEncMode(true, true); //make sure encoder position locking is enabled
    	Robot.encoderSubsystem.setTargetPositionDeg(45.0);
    	Timer.delay(0.5);
    	Robot.encoderSubsystem.setTargetPositionDeg(SmartDashboard.getNumber("Launch Angle (Test)")); //grab the calculated launch angle
    	Robot.cannonSubsystem.motorLaunch(); //start motors
    	Timer.delay(2);
    	Robot.pusherSubsystem.positionCenter(); //push ball
    	Timer.delay(4);
    	Robot.pusherSubsystem.positionReverse(); //retract pusher
    	Robot.visionSubsystem.disableProcessing(); //disable vision processing to free up resources
    	Robot.encoderSubsystem.setEncMode(false, true); //disable encoder locking to prepare for next pickup
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
