
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToggle extends Command {

	public boolean finished = false;
	String driveMode = "";
	
	/**
	 * Cycle through drive settings
	 * 0 = tank
	 * 1 = jonny
	 * 2 = tommy
	 */
    public DriveToggle() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrainSubsystem);
    }
    
    /**
     * Set drive to a specific setting
     * @param driveSetting Can be "tank", "jonny", "tommy"
     */
    public DriveToggle(String driveSetting) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrainSubsystem);
        driveMode = driveSetting;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	switch(driveMode) {
    	case "tank":
    		Robot.driveTrainSubsystem.toggleDriveSetting("tank");
    		break;
    	case "jonny":
    		Robot.driveTrainSubsystem.toggleDriveSetting("jonny");
    		break;
    	case "tommy":
    		Robot.driveTrainSubsystem.toggleDriveSetting("tommy");
    		break;
    	default:
    		Robot.driveTrainSubsystem.toggleDriveSetting();
    		break;
    	}
    	finished = true;
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
