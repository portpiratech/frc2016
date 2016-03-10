/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team4804.robot.commands;
import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VisionToggle extends Command {
    
	boolean processing = false;
	boolean altConstructor = false;
	
	/**
	 * Toggle vision mode -- processing enabled or disabled
	 */
    public VisionToggle() {
        requires(Robot.visionSubsystem);
    }
    
    /**
     * Set vision mode directly
     * @param process Should vision be processing?
     */
    public VisionToggle(boolean process) {
        requires(Robot.visionSubsystem);
        processing = process;
        altConstructor = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(altConstructor) {
    		//set vision mode directly
    		VisionSubsystem.visionProcessing = processing;
    	} else {
    		//toggle vision mode
	    	VisionSubsystem.visionProcessing = !VisionSubsystem.visionProcessing;
	        SmartDashboard.putBoolean("Vision Processing", VisionSubsystem.visionProcessing);
    	}
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
