
package org.usfirst.frc.team4804.robot.commands;

import org.usfirst.frc.team4804.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCommand extends CommandGroup {
	//defense names: "rough" "ramparts" "moat" "wall" "chival" "lowbar"
	//relative to center: "far right" "right" "center" "left" "far left"
	
	boolean lowbar = true;
	//String obstacle = "rough";
	String position = "center";
	
	boolean backwards = false;
	
	boolean highGoal = true;
	boolean lowGoal = false;
	
	//times for each driving segment (seconds)
	static double fwdTime = 2.0; //drive forward
	static double turnTime = 2.0; //turn robot if off-center
	static double turnTimeLow = 0.8; //turn robot if slightly off-center
	static double turnTimeHigh = 1.6; //turn robot if far off-center
	static double encMoveTime = 1.6; //move encoder to position
	static double targetTime = 2.0; //auto target time
	
    public AutonomousCommand() {
    	addSequential(new TargetingManual());
    	addSequential(new VisionToggle(false)); //disable vision processing
    	addSequential(new DriveToggle("tank")); //set drive into tank mode
    	addSequential(new EncoderSetting(true, true));
    	if(lowbar) {
    		addSequential(new CannonEncoderMove(-100, encMoveTime)); //move encoder to minimum angle (horizontal)
    		Timer.delay(encMoveTime); //wait "encMoveTime" seconds to make sure cannon is positioned
        	addParallel(new CannonEncoderMove(Robot.encoderSubsystem.POSITION_MIN_DEG, 0.2));
        	
        	if(!backwards) addSequential(new DriveCommand(fwdTime, 0.7, 0.7)); //drive at 60% speed for "fwdTime" seconds
        	if(backwards) addSequential(new DriveCommand(fwdTime, -0.7, -0.7));
        	Timer.delay(fwdTime);
    	} else {
    		//switch(obstacle) {}
        	if(!backwards) addSequential(new DriveCommand(fwdTime, 1.0, 1.0)); //drive at 60% speed for "fwdTime" seconds
        	if(backwards) addSequential(new DriveCommand(fwdTime, -1.0, -1.0));
        	Timer.delay(fwdTime);
    	}
    	
    	/*switch(position) {
    	case "far right":
    		addSequential(new DriveCommand(turnTimeHigh, -0.3, 0.3)); //turn left at 30% speed for "turnTime" seconds
        	Timer.delay(turnTimeHigh);
    		break;
    	case "right":
    		addSequential(new DriveCommand(turnTimeLow, -0.3, 0.3)); //turn left at 30% speed for "turnTime" seconds
        	Timer.delay(turnTimeLow);
    		break;
    	case "center":
    		//don't turn
    		break;
    	case "left":
    		addSequential(new DriveCommand(turnTimeLow, 0.3, -0.3)); //turn right at 30% speed for "turnTime" seconds
        	Timer.delay(turnTimeLow);
    		break;
    	case "far left":
    		addSequential(new DriveCommand(turnTimeHigh, 0.3, -0.3)); //turn right at 30% speed for "turnTime" seconds
        	Timer.delay(turnTimeHigh);
    		break;
    	}*/
    	
    	if(highGoal) {
	    	addSequential(new CannonEncoderMove(110, encMoveTime)); //move encoder to minimum angle (horizontal)
	    	Timer.delay(0.3);
	    	addSequential(new PositioningInit()); //put into position for auto-target
	    	addSequential(new DriveCommand(0.7, 0.5, -0.5));
	    	Timer.delay(encMoveTime); //wait "encMoveTime" seconds to make sure cannon is positioned
	    	
	    	addSequential(new VisionToggle(true)); //make sure vision is enabled
	    	addSequential(new TargetingAuto(targetTime)); //auto target for "targetTime" seconds
	    	Timer.delay(targetTime);
	    	
	    	addSequential(new Launch());
	    	Timer.delay(Launch.elapsedTime);
    	}
    	addSequential(new TargetingManual()); //switch back to manual target
    	//addSequential(new Launch()); //launch the ball
    	//Timer.delay(Launch.elapsedTime); //wait for launch to complete
    	
    	addSequential(new DriveCommand());
    	
    	//addSequential(new Load());
    	
    }
}
