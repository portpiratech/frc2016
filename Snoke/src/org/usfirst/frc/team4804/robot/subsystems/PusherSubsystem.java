
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonPusherMove;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PusherSubsystem extends Subsystem {
    
	//Uses Robot.pusher (Servo)
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new ExampleCommand());
    	setDefaultCommand(new CannonPusherMove());
    }
    
    public void positionForward() {
    	Robot.pusher.set(1.0);
    }
    
    public void positionReverse() {
    	Robot.pusher.set(0.0);
    }
    
    public void positionCenter() {
    	Robot.pusher.set(0.5);
    }

	public void positionJoystick(XboxController xbox) {
		Robot.pusher.set(0.5*(xbox.getRightStickXAxis()+1.0));
	}
}


