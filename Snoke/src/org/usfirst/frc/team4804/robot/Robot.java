
package org.usfirst.frc.team4804.robot;

import org.usfirst.frc.team4804.robot.commands.ExampleCommand;
import org.usfirst.frc.team4804.robot.subsystems.CannonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.PistonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.SwivelSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
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
  //MECHANICAL
   //Subsystems
	public static CannonSubsystem cannonSubsystem;
	public static PistonSubsystem pistonSubsystem;
	public static DriveTrainSubsystem driveTrainSubsystem;
	public static SwivelSubsystem swivelSubsystem;
	public static EncoderSubsystem encoderSubsystem;
	public static VisionSubsystem visionSubsystem;
	
   //Other classes
	public static Vision vision;
	public static OI oi;
	
   //Motor controllers and other objects
	public static Talon tankDriveLeftOld;
	public static Talon tankDriveRightOld;
	public static CANTalon tankDriveLeft;
	public static CANTalon tankDriveRight;
	public static CANTalon cannonLauncherMotors;
	public static CANTalon cannonEncoderMotor;
	public static CANTalon cannonSwivelMotor;
	public static Compressor cannonCompressor;
	
   //Switching robot mode
	public static RobotModes currentMode = RobotModes.NEW_ROBOT_MODE;
	
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
           //Subsystems
        	cannonSubsystem = new CannonSubsystem();
        	pistonSubsystem = new PistonSubsystem();
        	swivelSubsystem = new SwivelSubsystem();
        	encoderSubsystem = new EncoderSubsystem();
        	driveTrainSubsystem = new DriveTrainSubsystem();
        	visionSubsystem = new VisionSubsystem();
        	
           //Motors controllers and objects
        	cannonLauncherMotors = new CANTalon(OI.CANNON_LAUNCHER_ID);
        	cannonCompressor = new Compressor(OI.PCM_ID);
        	cannonCompressor.setClosedLoopControl(true);
        	cannonEncoderMotor = new CANTalon(OI.CANNON_ENCODER_ID);
        	tankDriveRight = new CANTalon(OI.NEW_TANKDRIVE_RIGHT_ID);
        	tankDriveLeft = new CANTalon(OI.NEW_TANKDRIVE_LEFT_ID);
        	cannonSwivelMotor = new CANTalon(OI.CANNON_SWIVEL_MOTOR_CHANNEL);
        	
           //sensors
        	/*limitLeft = new DigitalInput(OI.LIMIT_LEFT_ID);
        	limitRight = new DigitalInput(OI.LIMIT_RIGHT_ID);
        	limitBottom = new DigitalInput(OI.LIMIT_TOP_ID);*/
        	
           //SmartDashboard inputs? Need to test these
        	/*DriveTrainSubsystem.DRIVE_SPEED = (double)SmartDashboard.getNumber("Drive Speed Max", DriveTrainSubsystem.DRIVE_SPEED);
        	CannonSubsystem.LOAD_SPEED = (double)SmartDashboard.getNumber("Cannon Load Speed", CannonSubsystem.LOAD_SPEED);
        	CannonSubsystem.LAUNCH_SPEED = (double)SmartDashboard.getNumber("Cannon Launch Speed", CannonSubsystem.LAUNCH_SPEED);*/
        	break;
        	
        case OLD_TALON_TANK_MODE:
           //Motors controllers and objects
        	tankDriveLeftOld = new Talon(1);
            tankDriveRightOld = new Talon(0);
        	break;
        }
        
       //other important classes
        vision = new Vision();
        oi = new OI();
        
       //instantiate the command used for the autonomous period
        autonomousCommand = new ExampleCommand();
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
        SmartDashboard.putNumber("Drive Speed Max", DriveTrainSubsystem.driveSpeed);
    	SmartDashboard.putNumber("Cannon Load Speed", CannonSubsystem.LOAD_SPEED);
    	SmartDashboard.putNumber("Cannon Launch Speed", CannonSubsystem.LAUNCH_SPEED);
    	
    	//SmartDashboard.putNumber("setEncPosition", Robot.cannonEncoderMotor.getEncPosition());
    	SmartDashboard.putNumber("Encoder max speed", Robot.encoderSubsystem.SPEED_MAX);
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
        DriveTrainSubsystem.driveSpeed = (double)SmartDashboard.getNumber("Drive Speed Max");
    	CannonSubsystem.LOAD_SPEED = (double)SmartDashboard.getNumber("Cannon Load Speed");
    	CannonSubsystem.LAUNCH_SPEED = (double)SmartDashboard.getNumber("Cannon Launch Speed");
    	
    	//SmartDashboard.putNumber("setEncPosition", Robot.cannonEncoderMotor.getEncPosition());
    	//Robot.cannonEncoderMotor.setEncPosition((int)SmartDashboard.getNumber("setEncPosition"));
    	Robot.encoderSubsystem.SPEED_MAX = (double)SmartDashboard.getNumber("Encoder max speed");
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}

