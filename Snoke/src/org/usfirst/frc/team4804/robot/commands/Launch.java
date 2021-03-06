
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Launch the ball (only call after auto-positioning has been completed)
 */
public class Launch extends Command {
	
	boolean finished = true;
	
	//times for each delay segment (seconds)
	static double encMoveWait = 0.5; //wait for encoder to move to position
	static double motorWait = 2.0; //wait for motors to spin up
	static double pushWait = 2.5; //wait for ball to be pushed
	static double elapsedTime = encMoveWait + motorWait + pushWait;
	
    public Launch() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cannonSubsystem);
        requires(Robot.pusherSubsystem);
        requires(Robot.encoderSubsystem);
        requires(Robot.visionSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setInterruptible(true);
    	
    	Robot.pusherSubsystem.positionReverse(); //make sure pusher is retracted
    	SmartDashboard.putNumber("Launch Angle", 50.0);
    	Robot.visionSubsystem.enableProcessing(); //enable vision processing so angle can be calculated
    	Robot.encoderSubsystem.setEncMode(true, true); //make sure encoder position locking is enabled
    	Robot.encoderSubsystem.setTargetPositionDeg(45.0);
    	Timer.delay(encMoveWait);
    	if(SmartDashboard.getNumber("Launch Angle") < 120) {
    		Robot.encoderSubsystem.setTargetPositionDeg(SmartDashboard.getNumber("Launch Angle")); //grab the calculated launch angle
    	} else {
    		Robot.encoderSubsystem.setTargetPositionDeg(50.0);
    	}
    	Robot.cannonSubsystem.motorLaunch(); //start motors
    	Timer.delay(motorWait);
    	Robot.pusherSubsystem.positionCenter(); //push ball
    	Timer.delay(pushWait);
    	Robot.pusherSubsystem.positionReverse(); //retract pusher
    	Robot.visionSubsystem.disableProcessing(); //disable vision processing to free up resources
    	Robot.encoderSubsystem.setEncMode(false, true); //disable encoder locking to prepare for next pickup
    	Robot.cannonSubsystem.motorStop(); //stop motors
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
    	finished = true;
    	isFinished();
    }
}
