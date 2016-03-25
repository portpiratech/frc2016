
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PusherSubsystem extends Subsystem {
    
	//Uses Robot.pusher (Servo)
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new CannonPusherMove());
    }
    
    public void positionForward() {
    	SmartDashboard.putNumber("Pusher Target", 0.0);
    	Robot.pusher.set(0.0);
    }
    
    public void positionReverse() {
    	SmartDashboard.putNumber("Pusher Target", 1.0);
    	Robot.pusher.set(1.0);
    }
    
    public void positionCenter() {
    	SmartDashboard.putNumber("Pusher Target", 0.5);
    	Robot.pusher.set(0.5);
    }

	public void positionJoystick(XboxController xbox) {
		double position = xbox.getRightStickXAxis();
		SmartDashboard.putNumber("Pusher Target", position);
		if(position < 0.1) {
			position = 0;
		}
		Robot.pusher.set(position);
	}
}


