
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.RobotModes;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CannonSubsystem extends Subsystem {
    
	// Put important variables and objects for this subsystem here.
	
	// Wheel launcher variables
	public static double LOAD_SPEED = 0.45;
	public static double LAUNCH_SPEED = -1.0;
	
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    
    // Loading/Launching the ball
    public void motorLoad() {
    	setMotor(-LOAD_SPEED);
    	/*setMotor("L", LOAD_SPEED_L);
    	setMotor("R", LOAD_SPEED_R);*/
    }
    public void motorLoad(XboxController xbox) {
    	setMotor(-LOAD_SPEED*Math.abs(xbox.getRightStickYAxis()));
    }
    
    public void motorLaunch() {
    	setMotor(-LAUNCH_SPEED);
    	/*setMotor("L", LAUNCH_SPEED_L);
    	setMotor("R", LAUNCH_SPEED_R);*/
    }
    public void motorLaunch(XboxController xbox) {
    	setMotor(-LAUNCH_SPEED*Math.abs(xbox.getRightStickYAxis()));
    	
    }
    
    public void motorStop() {
    	setMotor(0);
    	/*setMotor("L", 0);
    	setMotor("R", 0);*/
    }
    
    public void setMotor(double speed) { //setMotor(String side, double speed)
    // single CANTalon:
    	switch(Robot.currentMode) {
    	case NEW_ROBOT_MODE:
    		Robot.cannonLauncherMotors.set(speed);
    		break;
    	case TEST_ROBOT_MODE:
    		Robot.cannonLauncherMotorsTest.set(speed);
    		break;
    	}
    	
    	SmartDashboard.putNumber("Cannon Launcher Speed", speed);
    	
    // independent CANTalons:
    /*	// side "L" = left
    	// side "R" = right1
    		
    	switch(side) {
    		case "L": Robot.cannonLauncherMotorLeft.set(speed); // check if this is the right method
    		case "R": Robot.cannonLauncherMotorRight.set(speed);
    	} */
    }
    
    
    // Positioning
    public void setCannonPosition(double position) {
    	//Robot.cannonTiltMotor.set(position); // need to set CANTalon to correct mode
    }
    
    
    // Set the default command for a subsystem here.
    public void initDefaultCommand() {
        // setDefaultCommand(new ExampleCommand());
    }
}


