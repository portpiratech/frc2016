
package org.usfirst.frc.team4804.robot;

import org.usfirst.frc.team4804.robot.commands.AutonomousCommand;
import org.usfirst.frc.team4804.robot.subsystems.BallDetectorSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.CannonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.PistonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.PusherSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.SwivelSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
   //Switching robot mode: NEW_ROBOT_MODE, TEST_ROBOT_MODE
	public static RobotModes currentMode = RobotModes.NEW_ROBOT_MODE;
	
   //subsystem debug enable/disable settings
	boolean vision_ = true;
	boolean pusher_ = true;
	boolean drive_ = true;
	boolean encoder_ = true;
	boolean swivel_ = false;
	boolean cannon_ = true;
	boolean piston_ = false;
	boolean detector_ = true;
		
  //MECHANICAL
   //Subsystems
	public static CannonSubsystem cannonSubsystem;
	public static PistonSubsystem pistonSubsystem;
	public static DriveTrainSubsystem driveTrainSubsystem;
	public static SwivelSubsystem swivelSubsystem;
	public static EncoderSubsystem encoderSubsystem;
	public static PusherSubsystem pusherSubsystem;
	public static BallDetectorSubsystem ballDetectorSubsystem;
	public static VisionSubsystem visionSubsystem;
	
   //Other classes
	public static Vision vision;
	public static OI oi;
	
   //Motor controllers and other objects
	public static Talon tankDriveLeftOld;
	public static Talon tankDriveRightOld;
	public static Talon tankDriveLeftTest;
	public static Talon tankDriveRightTest;
	public static Talon cannonLauncherMotorsTest;
	public static Servo pusher;
	public static CANTalon tankDriveLeft;
	public static CANTalon tankDriveRight;
	public static CANTalon cannonLauncherMotors;
	public static CANTalon cannonEncoderMotor;
	public static CANTalon cannonSwivelMotor;
	public static Compressor cannonCompressor;
	public static DigitalInput ballDetectorLim;
	
   //Autonomous command
    Command autonomousCommand;
    
   //Constructor
    public Robot() {
        //server = CameraServer.getInstance();
        //server.setQuality(50);
        ////the camera name (ex "cam0") can be found through the roborio web interface
        //server.startAutomaticCapture("cam0");
    }
    
    /**
     * start up automatic capture you should see the video stream from the
     * webcam in your FRC PC Dashboard.
     */
    

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
        switch (currentMode){
        
        case NEW_ROBOT_MODE:
            //Motors controllers and objects
        	if (drive_) {
        		tankDriveRight = new CANTalon(OI.TANKDRIVE_RIGHT_ID);
            	tankDriveLeft = new CANTalon(OI.TANKDRIVE_LEFT_ID);
        	}
        	if (cannon_) {
        		cannonLauncherMotors = new CANTalon(OI.CANNON_LAUNCHER_ID);
        	}
        	if (piston_) {
        		cannonCompressor = new Compressor(OI.PCM_ID);
        		cannonCompressor.setClosedLoopControl(true);
        	}
        	if (encoder_) {
        		cannonEncoderMotor = new CANTalon(OI.CANNON_ENCODER_ID);
        	}
        	if (swivel_) {
        		cannonSwivelMotor = new CANTalon(OI.CANNON_SWIVEL_MOTOR_ID);
        	}
        	if (pusher_) {
        		pusher = new Servo(OI.PUSHER_SERVO_CHANNEL);
        	}
        	if (detector_) {
        		ballDetectorLim = new DigitalInput(OI.DETECTOR_LIM);
        	}
        	
        	//Subsystems
        	if (drive_) {
        		driveTrainSubsystem = new DriveTrainSubsystem();
        	}
        	if (cannon_) {
        		cannonSubsystem = new CannonSubsystem();
        	}
        	if (piston_) {
        		pistonSubsystem = new PistonSubsystem();
        	}
        	if (encoder_) {
        		encoderSubsystem = new EncoderSubsystem();
        	}
        	if (swivel_) {
        		swivelSubsystem = new SwivelSubsystem();
        	}
        	if (pusher_) {
        		pusherSubsystem = new PusherSubsystem();
        	}
        	if (detector_) {
        		ballDetectorSubsystem = new BallDetectorSubsystem();
        	}
        	if (vision_) {
        		visionSubsystem = new VisionSubsystem();
        	}
        	break;
        	
        case TEST_ROBOT_MODE:
        	//Motor controllers and objects
        	if (drive_) tankDriveLeftTest = new Talon(OI.TEST_TANKDRIVE_LEFT_CHANNEL);
        	if (drive_) tankDriveRightTest = new Talon(OI.TEST_TANKDRIVE_RIGHT_CHANNEL);
        	if (cannon_) cannonLauncherMotorsTest = new Talon(OI.TEST_LAUNCHER_CHANNEL);
        	if (encoder_) cannonEncoderMotor = new CANTalon(OI.TEST_CANNON_ENCODER_ID);
        	if (swivel_) cannonSwivelMotor = new CANTalon(OI.TEST_CANNON_SWIVEL_ID);
        	if (pusher_) pusher = new Servo(OI.TEST_PUSHER_SERVO_CHANNEL);
        	
        	//Subsystems:
        	if (cannon_) cannonSubsystem = new CannonSubsystem();
        	if (swivel_) swivelSubsystem = new SwivelSubsystem();
        	if (encoder_) encoderSubsystem = new EncoderSubsystem();
        	if (drive_) driveTrainSubsystem = new DriveTrainSubsystem();
        	if (pusher_) pusherSubsystem = new PusherSubsystem();
        	if (vision_) visionSubsystem = new VisionSubsystem();
        	break;
        }
        
       //other important classes
        vision = new Vision();
        oi = new OI();
        
       //instantiate the command used for the autonomous period
        autonomousCommand = new AutonomousCommand();
    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        if(drive_) {
        	SmartDashboard.putNumber("Drive Speed Max", DriveTrainSubsystem.driveSpeed);
        }
        
        if(cannon_) {
        	SmartDashboard.putNumber("Cannon Load Speed", CannonSubsystem.LOAD_SPEED);
        	SmartDashboard.putNumber("Cannon Launch Speed", CannonSubsystem.LAUNCH_SPEED);
    	}
        
    	if(encoder_) {
	    	//SmartDashboard.putNumber("setEncPosition", Robot.cannonEncoderMotor.getEncPosition());
	    	SmartDashboard.putNumber("Encoder max speed", Robot.encoderSubsystem.SPEED_MAX);
	    	SmartDashboard.putBoolean("Encoder auto?", Robot.encoderSubsystem.encPID);
    	}
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){
       //disable rumble
    	Robot.oi.operatorController.setRumble(RumbleType.kLeftRumble, (float)0);
    	Robot.oi.operatorController.setRumble(RumbleType.kRightRumble, (float)0);
    	Robot.oi.driverController.setRumble(RumbleType.kLeftRumble, (float)0);
    	Robot.oi.driverController.setRumble(RumbleType.kRightRumble, (float)0);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        if(drive_){
        	DriveTrainSubsystem.driveSpeed = (double)SmartDashboard.getNumber("Drive Speed Max");
        }
        
        if(cannon_){
        	CannonSubsystem.LOAD_SPEED = (double)SmartDashboard.getNumber("Cannon Load Speed");
        	CannonSubsystem.LAUNCH_SPEED = (double)SmartDashboard.getNumber("Cannon Launch Speed");
        }
        
    	if(encoder_) {
	    	//SmartDashboard.putNumber("EncPosition", Robot.cannonEncoderMotor.getEncPosition());
	    	//Robot.cannonEncoderMotor.setEncPosition((int)SmartDashboard.getNumber("setEncPosition"));
	    	Robot.encoderSubsystem.SPEED_MAX = (double)SmartDashboard.getNumber("Encoder max speed");
	    	Robot.encoderSubsystem.encPID = (boolean)SmartDashboard.getBoolean("Encoder auto?");
	    	SmartDashboard.putNumber("EncPosition", Robot.cannonEncoderMotor.getEncPosition());
	    	SmartDashboard.putNumber("EncVelocity", Robot.cannonEncoderMotor.getEncVelocity());
	    	SmartDashboard.putBoolean("Enc Reverse Lim", Robot.cannonEncoderMotor.isRevLimitSwitchClosed());
	    	SmartDashboard.putBoolean("Enc Forward Lim", Robot.cannonEncoderMotor.isFwdLimitSwitchClosed());
    	}
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}

