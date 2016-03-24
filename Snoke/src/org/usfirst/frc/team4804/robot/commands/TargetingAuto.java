package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Switch the robot into auto targeting mode
 */
public class TargetingAuto extends Command {
	
	boolean drive = true;
	boolean encoder = true;
	boolean swivel = false;
	
	boolean finished = false;
	boolean autonomous = false;
	double secs, startTimeMs;
	
    public TargetingAuto() {
    	if(drive) {
	        requires(Robot.driveTrainSubsystem);
    	}
    	if(encoder) {
	    	requires(Robot.encoderSubsystem);
    	}
    }
    
    public TargetingAuto(double seconds) {
    	if(drive) {
	        requires(Robot.driveTrainSubsystem);
    	}
    	if(encoder) {
	    	requires(Robot.encoderSubsystem);
    	}
    	autonomous = true;
    	secs = seconds;
    	startTimeMs = System.currentTimeMillis();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.visionSubsystem.enableProcessing(); //make sure vision processing is enabled
    	if(drive) {
    		Robot.driveTrainSubsystem.enablePID(true);
	        SmartDashboard.putNumber("Drive const-Proportional (p)", Robot.driveTrainSubsystem.p);
	    	SmartDashboard.putNumber("Drive const-Integral (i)", Robot.driveTrainSubsystem.i);
	    	SmartDashboard.putNumber("Drive const-Derivative (d)", Robot.driveTrainSubsystem.d);
    	}
    	if(encoder) {
    		Robot.encoderSubsystem.setEncMode(true, true); //set encoder into locking and manual target setting mode
	    	SmartDashboard.putNumber("Enc const-Proportional (p)", Robot.encoderSubsystem.p);
	    	SmartDashboard.putNumber("Enc const-Integral (i)", Robot.encoderSubsystem.i);
	    	SmartDashboard.putNumber("Enc const-Derivative (d)", Robot.encoderSubsystem.d);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(drive) {
    		Robot.driveTrainSubsystem.updatePID();
	    }
	    if(encoder) {	
	    	Robot.encoderSubsystem.move(Robot.oi.operatorController);
	    	Robot.encoderSubsystem.updatePID();
	    }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(autonomous && System.currentTimeMillis() - startTimeMs >= 1000*secs) {
    		finished = true;
    	}
        return finished;
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
