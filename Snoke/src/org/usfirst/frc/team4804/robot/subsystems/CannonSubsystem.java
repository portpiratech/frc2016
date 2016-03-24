
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CannonSubsystem extends Subsystem {
    
	// Put important variables and objects for this subsystem here.
	
	// Wheel launcher variables
	public static double LOAD_SPEED = -0.55;
	public static double LAUNCH_SPEED = 1.0;
	
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    
    // Loading/Launching the ball
    public void motorLoad() {
    	setMotor(-LOAD_SPEED);
    }
    public void motorLoad(XboxController xbox) {
    	setMotor(-LOAD_SPEED*Math.abs(xbox.getRightStickYAxis()));
    }
    
    public void motorLaunch() {
    	setMotor(-LAUNCH_SPEED);
    }
    public void motorLaunch(XboxController xbox) {
    	setMotor(-LAUNCH_SPEED*Math.abs(xbox.getRightStickYAxis()));
    }
    
    public void motorStop() {
    	setMotor(0);
    }
    
    public void setMotor(double speed) {
    	switch(Robot.currentMode) {
    	case NEW_ROBOT_MODE:
    		Robot.cannonLauncherMotors.set(speed);
    		Robot.cannonLauncherMotors_2.set(-speed);
    		break;
    	case TEST_ROBOT_MODE:
    		Robot.cannonLauncherMotorsTest.set(speed);
    		Robot.cannonLauncherMotors_2.set(-speed);
    		break;
    	}
    	
    	SmartDashboard.putNumber("Cannon Launcher Speed", speed);
    }
    
    // Set the default command for a subsystem here.
    public void initDefaultCommand() {
        // setDefaultCommand(new ExampleCommand());
    }
}


