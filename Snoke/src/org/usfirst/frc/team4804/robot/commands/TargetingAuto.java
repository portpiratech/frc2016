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
	
    public TargetingAuto() {
    	if(drive) {
	        requires(Robot.driveTrainSubsystem);
	        SmartDashboard.putNumber("Drive const-Proportional (p)", Robot.driveTrainSubsystem.p);
	    	SmartDashboard.putNumber("Drive const-Integral (i)", Robot.driveTrainSubsystem.i);
	    	SmartDashboard.putNumber("Drive const-Derivative (d)", Robot.driveTrainSubsystem.d);
    	}
    	
    	if(encoder) {
	    	requires(Robot.encoderSubsystem);
	    	SmartDashboard.putNumber("Enc const-Proportional (p)", Robot.encoderSubsystem.p);
	    	SmartDashboard.putNumber("Enc const-Integral (i)", Robot.encoderSubsystem.i);
	    	SmartDashboard.putNumber("Enc const-Derivative (d)", Robot.encoderSubsystem.d);
    	}
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.visionSubsystem.enableProcessing(); //make sure vision processing is enabled
    	if(drive) {
    		Robot.driveTrainSubsystem.enablePID(true);
    	}
    	if(encoder) {
    		Robot.encoderSubsystem.setEncMode(true, true); //set encoder into locking and manual target setting mode
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(drive) {
    		Robot.driveTrainSubsystem.updatePID();
	    	/*Robot.driveTrainSubsystem.p = SmartDashboard.getNumber("Drive const-Proportional (p)");
	    	Robot.driveTrainSubsystem.i = SmartDashboard.getNumber("Drive const-Integral (i)");
	    	Robot.driveTrainSubsystem.d = SmartDashboard.getNumber("Drive const-Derivative (d)");
	    	Robot.driveTrainSubsystem.getPIDController().setPID(Robot.driveTrainSubsystem.p, Robot.driveTrainSubsystem.i, Robot.driveTrainSubsystem.d);*/
    	}
	    if(encoder) {	
	    	Robot.encoderSubsystem.move(Robot.oi.operatorController);
	    	Robot.encoderSubsystem.updatePID();
	    	/*Robot.encoderSubsystem.p = SmartDashboard.getNumber("Enc const-Proportional (p)");
	    	Robot.encoderSubsystem.i = SmartDashboard.getNumber("Enc const-Integral (i)");
	    	Robot.encoderSubsystem.d = SmartDashboard.getNumber("Enc const-Derivative (d)");
	    	Robot.encoderSubsystem.setPID(Robot.encoderSubsystem.p, Robot.encoderSubsystem.i, Robot.encoderSubsystem.d);*/
	    }
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
