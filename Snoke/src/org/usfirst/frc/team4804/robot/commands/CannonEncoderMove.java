
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CannonEncoderMove extends Command {
	
	boolean finished = false;
	boolean autonomous = false, autonomous2 = false;
	double targetPositionDeg, secs, startTimeMs, encSpeed;
	
    public CannonEncoderMove() {
        // Use requires() here to declare subsystem dependencies
    	//Default control
        requires(Robot.encoderSubsystem);
        setInterruptible(true);
    }
    
    public CannonEncoderMove(double targetPositionDegrees, double seconds) {
    	//Direct position control
    	requires(Robot.encoderSubsystem);
    	autonomous = true;
    	targetPositionDeg = targetPositionDegrees;
    	secs = seconds;
    }
    
    public CannonEncoderMove(double speed, double seconds, boolean percentvbus) {
    	//Direct velocity control
    	requires(Robot.encoderSubsystem);
    	autonomous2 = true;
    	secs = seconds;
    	encSpeed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startTimeMs = System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//SmartDashboard.putBoolean("Encoder auto?", Robot.encoderSubsystem.encPID);
    	SmartDashboard.putBoolean("Enc manualTarget", Robot.encoderSubsystem.manualTarget);
    	if(!autonomous && !autonomous2) {
    		Robot.encoderSubsystem.move(Robot.oi.operatorController);
    		//System.out.println("Move Called");
    	}
    	if(autonomous) {
    		Robot.encoderSubsystem.setEncMode(true, true);
    		Robot.encoderSubsystem.setTargetPositionDeg(targetPositionDeg);
    	}
    	if(autonomous2) {
    		Robot.encoderSubsystem.setEncMode(false, true);
    		Robot.encoderSubsystem.setMotorSpeed(encSpeed);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(autonomous || autonomous2) {
    		//check time limit for command
    		if(System.currentTimeMillis() - startTimeMs >= 1000*secs) {
    			finished = true;
    		}
    	}
    	if(autonomous2) {
    		//check limit switches
    		if(Robot.cannonEncoderMotor.isFwdLimitSwitchClosed() || Robot.cannonEncoderMotor.isRevLimitSwitchClosed()) {
    			Robot.encoderSubsystem.setMotorSpeed(0);
    			finished = true;
    		}
    	}
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	if(autonomous2) {
    		//check limit switches
    		if(Robot.cannonEncoderMotor.isFwdLimitSwitchClosed() || Robot.cannonEncoderMotor.isRevLimitSwitchClosed()) {
    			Robot.encoderSubsystem.setMotorSpeed(0);
    		}
    	}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
