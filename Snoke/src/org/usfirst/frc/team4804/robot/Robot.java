
package org.usfirst.frc.team4804.robot;

import org.usfirst.frc.team4804.robot.commands.ExampleCommand;
import org.usfirst.frc.team4804.robot.subsystems.CannonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.PistonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.ServoSubsystem;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	//public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	
	public static CannonSubsystem cannonSubsystem;
	public static PistonSubsystem pistonSubsystem;
	public static DriveTrainSubsystem driveTrainSubsystem;
	public static ServoSubsystem servoSubsystem;
	public static EncoderSubsystem encoderSubsystem;
	
	public static OI oi;
	
	public static Talon tankDriveLeftOld;
	public static Talon tankDriveRightOld;
	public static CANTalon tankDriveLeft;
	public static CANTalon tankDriveRight;
	public static CANTalon cannonLauncherMotors;
	//public static CANTalon cannonLauncherMotorRight;
	//public static CANTalon cannonLauncherMotorLeft;
	public static CANTalon cannonEncoderMotor;
	public static Compressor cannonCompressor;

	public static RobotModes currentMode = RobotModes.NEW_ROBOT_MODE;
	
    Command autonomousCommand;
    
    CameraServer server;
    
    public Robot() {
        server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
    }
    
    /**
     * start up automatic capture you should see the video stream from the
     * webcam in your FRC PC Dashboard.
     */
    public void operatorControl() {

        while (isOperatorControl() && isEnabled()) {
            // robot code here!
            Timer.delay(0.005);		// wait for a motor update time
        }
    }
    

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
        switch (currentMode){
        
        case NEW_ROBOT_MODE:
        	cannonSubsystem = new CannonSubsystem();
        	pistonSubsystem = new PistonSubsystem();
        	servoSubsystem = new ServoSubsystem();
        	encoderSubsystem = new EncoderSubsystem();
        	
        	cannonLauncherMotors = new CANTalon(OI.CANNON_LAUNCHER_ID); //2
        	//cannonLauncherMotorRight = new CANTalon(OI.CANNON_LAUNCHER_RIGHT_ID); //2
        	//cannonLauncherMotorLeft = new CANTalon(OI.CANNON_LAUNCHER_LEFT_ID); //3
        	
        	cannonCompressor = new Compressor(OI.PCM_ID);
        	cannonCompressor.setClosedLoopControl(true);
        	
        	driveTrainSubsystem = new DriveTrainSubsystem();
        	tankDriveRight = new CANTalon(OI.NEW_TANKDRIVE_RIGHT_ID);
        	tankDriveLeft = new CANTalon(OI.NEW_TANKDRIVE_LEFT_ID);
        	
        	cannonEncoderMotor = new CANTalon(OI.CANNON_ENCODER_ID);
        	break;
        	
        case OLD_TALON_TANK_MODE:
        	tankDriveLeftOld = new Talon(1);
            tankDriveRightOld = new Talon(0);
        	break;
        }
        
        oi = new OI();
        
     // instantiate the command used for the autonomous period
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
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
        
    }
    
    
}

