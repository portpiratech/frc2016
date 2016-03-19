package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.VisionDisplay;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
public class VisionSubsystem extends Subsystem {
	
	public double cameraHeightMeters = 0.2; //relative to floor
	
	public static boolean visionProcessing;
	//long lastFrameProcessTimeMs;
	//long captureIntervalMs;
	
	public VisionSubsystem() {
		visionProcessing = false;
//		lastFrameProcessTimeMs = 0;
//		captureIntervalMs = 200;
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new VisionDisplay());
	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */ 
	public void log() {
	}
	
	public void frameProcess() {
		for(int i = 0; i<10; i++) {
		//if(System.currentTimeMillis() - lastFrameProcessTimeMs >= captureIntervalMs) {
			Robot.vision.frameProcess();
			Timer.delay(0.1);
			//lastFrameProcessTimeMs = System.currentTimeMillis();
		//}
		}
	}
	
	public void frameAutoDisplay() {
		Robot.vision.frameAutoDisplay();
	}
}
