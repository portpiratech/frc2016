
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCommand extends CommandGroup {
	//times for each driving segment (seconds)
	static double fwdTime = 2.0; //drive forward
	static double turnTime = 1.0; //turn robot
	static double encMoveTime = 1.0; //move encoder to position
	static double targetTime = 1.5; //auto target time
	
    public AutonomousCommand() {
    	addSequential(new TargetingManual());
    	addSequential(new VisionToggle(false)); //disable vision processing
    	addSequential(new DriveToggle("tank")); //set drive into tank mode
    	addSequential(new EncoderSetting(true, true));
    	addSequential(new CannonEncoderMove(-1.0, encMoveTime)); //move encoder to minimum angle (horizontal)
    	Timer.delay(encMoveTime); //wait "encMoveTime" seconds to make sure cannon is positioned
    	addParallel(new CannonEncoderMove(Robot.encoderSubsystem.POSITION_MIN_DEG, 0.2));
    	
    	addSequential(new DriveCommand(fwdTime, 1.0, 1.0)); //drive at 60% speed for "fwdTime" seconds
    	Timer.delay(fwdTime);
    	
    	/*addSequential(new DriveCommand(turnTime, 0.3, -0.3)); //turn right at 30% speed for "turnTime" seconds
    	Timer.delay(turnTime);
    	
    	addSequential(new PositioningInit()); //put into position for auto-target
    	Timer.delay(encMoveTime); //wait "encMoveTime" seconds to make sure cannon is positioned
    	
    	addSequential(new VisionToggle(true)); //make sure vision is enabled
    	addSequential(new TargetingAuto(targetTime)); //auto target for "targetTime" seconds
    	Timer.delay(targetTime);*/
    	
    	addSequential(new TargetingManual()); //switch back to manual target
    	//addSequential(new Launch()); //launch the ball
    	//Timer.delay(Launch.elapsedTime); //wait for launch to complete
    	
    	addSequential(new DriveCommand());
    	
    	//addSequential(new Load());
    	
    }
}
