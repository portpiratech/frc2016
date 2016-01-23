package org.usfirst.frc.team4804.robot;

import org.usfirst.frc.team4804.robot.commands.CannonPistonFire;
import org.usfirst.frc.team4804.robot.commands.CannonWheelLaunch;
import org.usfirst.frc.team4804.robot.commands.CannonWheelLoad;
import org.usfirst.frc.team4804.robot.commands.CannonWheelStop;

import com.portpiratech.xbox360.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
	
	private static final int OPERATOR_CONTROLLER_PORT = 1;
	private static final int DRIVER_CONTROLLER_PORT = 0;
	public XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
	public XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);
    
    // Device IDs
    public static final int OLD_TANKDRIVE_LEFT_ID = 0; //Talon
	public static final int OLD_TANKDRIVE_RIGHT_ID = 1; //Talon
    public static final int NEW_TANKDRIVE_RIGHT_ID = 3; //CAN Talon SRX
	public static final int NEW_TANKDRIVE_LEFT_ID = 4; //CAN Talon SRX
	public static final int CANNON_LAUNCHER_ID = 2; //CAN Talon SRX
	public static final int CANNON_TILT_MOTOR_ID = 5; //CAN Talon SRX
	//public static final int CANNON_LAUNCHER_RIGHT_ID = 2; //CAN Talon SRX
	//public static final int CANNON_LAUNCHER_LEFT_ID = 3; //CAN Talon SRX
	public static final int SOLENOID1_PORT1 = 0; //DoubleSolenoid
    public static final int SOLENOID1_PORT2 = 1; //DoubleSolenoid
    public static final int SOLENOID2_PORT1 = 2; //DoubleSolenoid
    public static final int SOLENOID2_PORT2 = 3; //DoubleSolenoid
    public static final int PCM_ID = 1; //Compressor/Pneumatics Control Module (used for DoubleSolenoids)
    
	
	public OI() {
        // Connect the buttons to commands
		
	// Driver: Tank drive
		
	// Operator: Cannon/piston controls
		operatorController.getLeftBumper().whenPressed(new CannonWheelLoad());
		operatorController.getRightBumper().whenPressed(new CannonWheelLaunch());
		operatorController.getYButton().whenPressed(new CannonWheelStop());
		
		operatorController.getXButton().whenPressed(new CannonPistonFire());
		
		//operatorController.getLeftStickYAxis().whenPressed()
		
		
	// SmartDashboard commands
		// SmartDashboard.putData("Cannon Piston Extend", new CannonPistonExtend());
	}
}

