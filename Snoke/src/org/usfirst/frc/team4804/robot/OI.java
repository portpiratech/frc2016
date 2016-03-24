package org.usfirst.frc.team4804.robot;

import org.usfirst.frc.team4804.robot.commands.CannonWheelStop;
import org.usfirst.frc.team4804.robot.commands.DriveToggle;
import org.usfirst.frc.team4804.robot.commands.EncoderSetting;
import org.usfirst.frc.team4804.robot.commands.Launch;
import org.usfirst.frc.team4804.robot.commands.Load;
import org.usfirst.frc.team4804.robot.commands.PositioningInit;
import org.usfirst.frc.team4804.robot.commands.TargetingAuto;
import org.usfirst.frc.team4804.robot.commands.TargetingManual;
import org.usfirst.frc.team4804.robot.commands.VisionToggle;

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
	
  //ROBOT MAP
	// Xbox Controllers
	private static final int OPERATOR_CONTROLLER_PORT = 1;
	private static final int DRIVER_CONTROLLER_PORT = 0;
	public XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
	public XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);
    
    // CAN Device IDs
    public static final int TANKDRIVE_RIGHT_ID = 6; //CAN Talon SRX
	public static final int TANKDRIVE_LEFT_ID = 3; //CAN Talon SRX
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
    public static final int PUSHER_SERVO_CHANNEL = 6; //Servo
    public static final int TEST_TANKDRIVE_LEFT_CHANNEL = 3; //TalonSR
    public static final int TEST_TANKDRIVE_RIGHT_CHANNEL = 4; //TalonSR
    public static final int TEST_LAUNCHER_CHANNEL = 1; //TalonSR???
    public static final int TEST_PUSHER_SERVO_CHANNEL = 6; //Servo
    
    // DIO (Digital Input/Output--on roboRIO) Channels
    public static final int DETECTOR_LIM = 0; // DigitalInput
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
			driverController.getStart().whenPressed(new TargetingAuto());
			driverController.getSelect().whenPressed(new TargetingManual());
			driverController.getAButton().whenPressed(new DriveToggle());
			driverController.getBButton().whenPressed(new VisionToggle());
			//left + right stick when in tank drive, left stick in tommy/jonny drive
			
		// Operator commands: Cannon/piston controls
			operatorController.getBButton().whenPressed(new EncoderSetting(!Robot.encoderSubsystem.encPID, true));
			operatorController.getXButton().whenPressed(new PositioningInit());
			operatorController.getLeftBumper().whenPressed(new Load());
			operatorController.getAButton().whenPressed(new Launch());
			operatorController.getYButton().whenPressed(new CannonWheelStop());
			//right stick to control encoder in manual mode
			
		// SmartDashboard commands?
			break;
		
		case TEST_ROBOT_MODE:
			driverController.getStart().whenPressed(new TargetingAuto());
			driverController.getSelect().whenPressed(new TargetingManual());
			driverController.getAButton().whenPressed(new DriveToggle());
			driverController.getBButton().whenPressed(new VisionToggle());
			
			//operatorController.getLeftBumper().whenPressed(new CannonWheelLoad());
			//operatorController.getRightBumper().whenPressed(new CannonWheelLaunch());
			//operatorController.getAButton().whenPressed(new CannonPusherCenter());
			//operatorController.getBButton().whenPressed(new CannonPusherReverse()); //have this automatically do that
			
			operatorController.getBButton().whenPressed(new EncoderSetting(!Robot.encoderSubsystem.encPID, true));
			operatorController.getXButton().whenPressed(new PositioningInit());
			operatorController.getLeftBumper().whenPressed(new Load());
			operatorController.getAButton().whenPressed(new Launch());
			operatorController.getYButton().whenPressed(new CannonWheelStop());
			
			/*SmartDashboard.putData("Pusher Forward", new CannonPusherForward());
			SmartDashboard.putData("Pusher Center", new CannonPusherCenter());
			SmartDashboard.putData("Pusher Reverse", new CannonPusherReverse());*/
			break;
		}
	}
}

