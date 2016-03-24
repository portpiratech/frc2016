
package org.usfirst.frc.team4804.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCommand extends CommandGroup {
    public AutonomousCommand() {
    	addSequential(new VisionToggle(false)); //disable vision processing
    	addSequential(new DriveToggle("tank")); //set drive into tank mode
    	addSequential(new CannonEncoderMove(0.0)); //move encoder to 0 degrees (horizontal)
    	Timer.delay(0.5); //wait half a second to make sure cannon is positioned
    	addSequential(new DriveCommand(5.0, 0.6, 0.6)); //drive at 60% speed for 5 seconds
    	addSequential(new DriveCommand(2.0, 0.3, -0.3)); //turn right at 30% speed for 3 seconds
    	addSequential(new PositioningInit()); //put into position for auto-target
    	Timer.delay(0.5); //wait half a second to make sure cannon is positioned
    	addSequential(new TargetingAuto(2.0)); //auto target for 2 seconds
    	addSequential(new Launch()); //launch the ball
    	
    	//Roughly 11 seconds should have elapsed?
    	
    	//addSequential(new Load());
    	
    }
}
