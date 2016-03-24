
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CannonEncoderMove extends Command {
	
	boolean finished = false;
	boolean autonomous = false;
	double targetPositionDeg;
	
    public CannonEncoderMove() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.encoderSubsystem);
        setInterruptible(true);
    }
    
    public CannonEncoderMove(double targetPositionDegrees) {
    	requires(Robot.encoderSubsystem);
    	autonomous = true;
    	targetPositionDeg = targetPositionDegrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//SmartDashboard.putBoolean("Encoder auto?", Robot.encoderSubsystem.encPID);
    	SmartDashboard.putBoolean("Enc manualTarget", Robot.encoderSubsystem.manualTarget);
    	if(!autonomous) {
    		Robot.encoderSubsystem.move(Robot.oi.operatorController);
    		System.out.println("Move Called");
    	}else{
    		Robot.encoderSubsystem.setEncMode(true, true);
    		Robot.encoderSubsystem.setTargetPositionDeg(targetPositionDeg);
    		finished = true;
        	
    	}


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
