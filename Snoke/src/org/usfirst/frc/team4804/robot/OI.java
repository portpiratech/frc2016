package org.usfirst.frc.team4804.robot;

import org.usfirst.frc.team4804.robot.commands.CameraUpdate;
import org.usfirst.frc.team4804.robot.commands.CannonPistonExtend;
import org.usfirst.frc.team4804.robot.commands.CannonPistonFire;
import org.usfirst.frc.team4804.robot.commands.CannonPistonRetract;
import org.usfirst.frc.team4804.robot.commands.CannonPusherCenter;
import org.usfirst.frc.team4804.robot.commands.CannonPusherForward;
import org.usfirst.frc.team4804.robot.commands.CannonPusherReverse;
import org.usfirst.frc.team4804.robot.commands.CannonWheelLaunch;
import org.usfirst.frc.team4804.robot.commands.CannonWheelLoad;
import org.usfirst.frc.team4804.robot.commands.CannonWheelStop;
import org.usfirst.frc.team4804.robot.commands.DriveToggle;
import org.usfirst.frc.team4804.robot.commands.TargetingAuto;
import org.usfirst.frc.team4804.robot.commands.TargetingManual;
import org.usfirst.frc.team4804.robot.commands.VisionToggle;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
  //ROBOT MAP
	// Xbox Controllers
	private static final int OPERATOR_CONTROLLER_PORT = 1;
	private static final int DRIVER_CONTROLLER_PORT = 0;
	public XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
	public XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);
    
    // CAN Device IDs
    public static final int OLD_TANKDRIVE_LEFT_ID = 0; //Talon
	public static final int OLD_TANKDRIVE_RIGHT_ID = 1; //Talon
    public static final int NEW_TANKDRIVE_RIGHT_ID = 6; //CAN Talon SRX
	public static final int NEW_TANKDRIVE_LEFT_ID = 3; //CAN Talon SRX
	public static final int CANNON_LAUNCHER_ID = 5; //CAN Talon SRX
	public static final int CANNON_ENCODER_ID = 2; //CAN Talon SRX
    public static final int CANNON_SWIVEL_MOTOR_ID = 4; //CAN Talon SRX
    public static final int PCM_ID = 1; //Compressor/Pneumatics Control Module (used for DoubleSolenoids)
    
    public static final int TEST_CANNON_ENCODER_ID = 10; //CAN Talon SRX
    public static final int TEST_CANNON_SWIVEL_ID = 11; //CAN Talon SRX
    
    // PCM (Pneumatics Control Module) Channels
    public static final int SOLENOID1_PORT1 = 0; //DoubleSolenoid
    public static final int SOLENOID1_PORT2 = 1; //DoubleSolenoid
    public static final int SOLENOID2_PORT1 = 2; //DoubleSolenoid
    public static final int SOLENOID2_PORT2 = 3; //DoubleSolenoid
    
    // PWM (Pulse Width Modulation--on roboRIO) Device Channels
    public static final int TEST_TANKDRIVE_LEFT_CHANNEL = 0; //TalonSR
    public static final int TEST_TANKDRIVE_RIGHT_CHANNEL = 1; //TalonSR
    public static final int TEST_LAUNCHER_CHANNEL = 2; //TalonSR???
    public static final int TEST_PUSHER_SERVO_CHANNEL = 4; //Servo
    
    // DIO (Digital Input/Output--on roboRIO) Channels
    /*public static final int CANNON_ENCODER_CHANNEL_A = 0; // DigitalInput
    public static final int CANNON_ENCODER_CHANNEL_B = 1; // DigitalInput
    public static final int LIMIT_LEFT_ID = 9;            // DigitalInput (limit switch for swivel)
    public static final int LIMIT_RIGHT_ID = 8;			  // DigitalInput
    public static final int LIMIT_CENTER_ID = 7;*/
	
  //CONSTRUCTOR/COMMANDS
	public OI() {
        // Connect the buttons to commands
		switch(Robot.currentMode) {
		case NEW_ROBOT_MODE:
		// Driver commands:
			driverController.getAButton().whenPressed(new DriveToggle());
			driverController.getBButton().whenPressed(new VisionToggle());
			
			driverController.getStart().whenPressed(new TargetingAuto());
			driverController.getSelect().whenPressed(new TargetingManual());
			//Left stick for driving, left + right stick when in tank drive
			
		// Operator commands: Cannon/piston controls
			operatorController.getLeftBumper().whenPressed(new CannonWheelLoad());
			operatorController.getRightBumper().whenPressed(new CannonWheelLaunch());
			
			operatorController.getYButton().whenPressed(new CannonWheelStop());
			operatorController.getXButton().whenPressed(new CannonPistonFire()); //automatic
			
			operatorController.getStart().whenPressed(new CannonPistonExtend()); // TODO: rotate the servo
			operatorController.getSelect().whenPressed(new CannonPistonRetract());
			operatorController.getAButton().whenPressed(new CameraUpdate());
			//right stick y axis used temporarily for controlling launch speed
			//left stick y axis used temporarily for the encoder motor
			
		// SmartDashboard commands?
			//SmartDashboard.putData("Cannon Piston Extend", new CannonPistonFire());
			break;
		
		case TEST_ROBOT_MODE:
			SmartDashboard.putData("Pusher Forward", new CannonPusherForward());
			SmartDashboard.putData("Pusher Center", new CannonPusherCenter());
			SmartDashboard.putData("Pusher Reverse", new CannonPusherReverse());
			break;
		
		case OLD_TALON_TANK_MODE:
			break;
		}
	}
}

