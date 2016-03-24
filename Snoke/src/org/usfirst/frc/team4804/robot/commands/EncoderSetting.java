/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team4804.robot.commands;
import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;


public class EncoderSetting extends Command {
    
	boolean encPID = false;
	boolean manualTarget = false;
    
    /**
     * Set encoder mode directly
     * @param PID Should encoder auto-lock (PID enabled), or use manual controls (joystick/speed)?
     * @param manual Should encoder target angle be set manually (through code/smartdashboard), or automatically (vision; not recommended)?
     */
    public EncoderSetting(boolean PID, boolean manual) {
        requires(Robot.encoderSubsystem);
        encPID = PID;
        manualTarget = manual;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.encoderSubsystem.setEncMode(encPID, manualTarget);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true; // Runs until interrupted, Xbox controller determines when finished
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
