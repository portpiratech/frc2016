
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CannonPistonFire extends Command {
	
	boolean finished = false;
	
    public CannonPistonFire() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.pistonSubsystem);
        requires(Robot.cannonSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Set cannons to launch mode, extend piston, wait, retract, stop cannons
    	SmartDashboard.putString("In execute:","CannonPistonFire");
    	
    	Robot.cannonSubsystem.motorLaunch();
    	Timer.delay(3); //delay for 3 seconds
    	
    	Robot.pistonSubsystem.extendLauncher();
    	Timer.delay(Robot.pistonSubsystem.firingDelay); //delay for 1 sec
    	
    	Robot.pistonSubsystem.retractLauncher();
    	Timer.delay(Robot.pistonSubsystem.firingDelay); //delay for 1 sec
    	
    	finished = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.pistonSubsystem.stopLauncher();
    	Robot.cannonSubsystem.motorStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
