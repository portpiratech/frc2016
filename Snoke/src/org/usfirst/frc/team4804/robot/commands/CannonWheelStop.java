
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CannonWheelStop extends Command {
	
    public CannonWheelStop() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cannonSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.cannonSubsystem.motorStop();
    	SmartDashboard.putNumber("Cannon Launcher Speed L", Robot.cannonLauncherMotorLeft.get());
    	SmartDashboard.putNumber("Cannon Launcher Speed R", Robot.cannonLauncherMotorRight.get());
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
    }
}