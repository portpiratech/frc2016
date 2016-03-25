
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveCommand extends Command {
	
	boolean finished = false;
	boolean autonomous = false;
	double secs, left, right, startTimeMs;
	
    public DriveCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrainSubsystem);
    }
    
    public DriveCommand(double seconds, double leftSpeed, double rightSpeed) {
    	requires(Robot.driveTrainSubsystem);
    	autonomous = true;
    	secs = seconds;
    	left = leftSpeed;
    	right = rightSpeed;
    	startTimeMs = System.currentTimeMillis();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(!autonomous) {
    		Robot.driveTrainSubsystem.drive(Robot.oi.driverController);
    	}else{
    		Robot.driveTrainSubsystem.tankDrive(left, right, -1);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(autonomous) {
    		if(System.currentTimeMillis() - startTimeMs >= 1000*secs) {
    			finished = true;
    		}
    	}
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
