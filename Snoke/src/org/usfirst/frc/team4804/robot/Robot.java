
package org.usfirst.frc.team4804.robot;

import org.usfirst.frc.team4804.robot.commands.ExampleCommand;
import org.usfirst.frc.team4804.robot.subsystems.CannonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.EncoderSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.PistonSubsystem;
import org.usfirst.frc.team4804.robot.subsystems.SwivelSubsystem;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
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
	
   //Other classes
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
    
   //sensors
    public static DigitalInput limitLeft;
    public static DigitalInput limitRight;
    
    /*
  //VISION
   //A structure to hold measurements of a particle
  	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
  		double PercentAreaToImageArea;
  		double Area;
  		double BoundingRectLeft;
  		double BoundingRectTop;
  		double BoundingRectRight;
  		double BoundingRectBottom;
  		
  		public int compareTo(ParticleReport r){
  			return (int)(r.Area - this.Area);
  		}
  		
  		public int compare(ParticleReport r1, ParticleReport r2){
  			return (int)(r1.Area - r2.Area);
  		}
  	};
  	
   //Structure to represent the scores for the various tests used for target identification
  	public class Scores {
  		double Area;
  		double Aspect;
  	};

   //Images
  	Image frame;
  	Image binaryFrame;
  	Image binaryFrameToDisplay;
  	int imaqError;
  	int session;
	
   //Constants
  	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(101, 64);	//Default hue range for yellow tote
  	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(88, 255);	//Default saturation range for yellow tote
  	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(134, 255);	//Default value range for yellow tote
  	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
  	double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 = 2.22
  	double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 = 1.4
  	double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
  	double VIEW_ANGLE = 49.4; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
  	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
  	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
  	Scores scores = new Scores();
    
   //Vision
    CameraServer server;
    //int session;
    //Image frame;
    
   //Constructor
    public Robot() {
        server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
    }*/
    
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
           //Subsystems
        	cannonSubsystem = new CannonSubsystem();
        	pistonSubsystem = new PistonSubsystem();
        	swivelSubsystem = new SwivelSubsystem();
        	encoderSubsystem = new EncoderSubsystem();
        	driveTrainSubsystem = new DriveTrainSubsystem();
        	
           //Motors controllers and objects
        	cannonLauncherMotors = new CANTalon(OI.CANNON_LAUNCHER_ID);
        	cannonCompressor = new Compressor(OI.PCM_ID);
        	cannonCompressor.setClosedLoopControl(true);
        	cannonEncoderMotor = new CANTalon(OI.CANNON_ENCODER_ID);
        	tankDriveRight = new CANTalon(OI.NEW_TANKDRIVE_RIGHT_ID);
        	tankDriveLeft = new CANTalon(OI.NEW_TANKDRIVE_LEFT_ID);
        	cannonSwivelMotor = new CANTalon(OI.CANNON_SWIVEL_MOTOR_CHANNEL);
        	
        	//sensors
        	limitLeft = new DigitalInput(OI.LIMIT_LEFT_ID);
        	limitRight = new DigitalInput(OI.LIMIT_RIGHT_ID);
        	
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
        
       //other classes
        oi = new OI();
        
       //instantiate the command used for the autonomous period
        autonomousCommand = new ExampleCommand();
       
      //VISION
       // create images
     	/*frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
     	binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
     	criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);

       //Put default values to SmartDashboard so fields will appear
     	SmartDashboard.putNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
     	SmartDashboard.putNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
     	SmartDashboard.putNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
     	SmartDashboard.putNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
     	SmartDashboard.putNumber("Tote val min", TOTE_VAL_RANGE.minValue);
     	SmartDashboard.putNumber("Tote val max", TOTE_VAL_RANGE.maxValue);
     	SmartDashboard.putNumber("Area min %", AREA_MINIMUM);
     			
     	session = NIVision.IMAQdxOpenCamera("cam0",
     	          NIVision.IMAQdxCameraControlMode.CameraControlModeController);
     	NIVision.IMAQdxConfigureGrab(session);*/
        
//    // camera stuff
//        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
//
//        // the camera name (ex "cam0") can be found through the roborio web interface
//        session = NIVision.IMAQdxOpenCamera("cam0",
//                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
//        NIVision.IMAQdxConfigureGrab(session);
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
        
        /*while (! && isEnabled()) {
			//read file in from disk. For this example to run you need to copy image.jpg from the SampleImages folder to the
			//directory shown below using FTP or SFTP: http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
			
			//NIVision.imaqReadFile(frame, "/home/lvuser/SampleImages/image.jpg");
			NIVision.IMAQdxGrab(session, frame, 1);

			//Update threshold values from SmartDashboard. For performance reasons it is recommended to remove this after calibration is finished.
			TOTE_HUE_RANGE.minValue = (int)SmartDashboard.getNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
			TOTE_HUE_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
			TOTE_SAT_RANGE.minValue = (int)SmartDashboard.getNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
			TOTE_SAT_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
			TOTE_VAL_RANGE.minValue = (int)SmartDashboard.getNumber("Tote val min", TOTE_VAL_RANGE.minValue);
			TOTE_VAL_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote val max", TOTE_VAL_RANGE.maxValue);

			//Threshold the image looking for yellow (tote color)
			NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);

			//Send particle count to dashboard
			int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Masked particles", numParticles);

			//Send masked image to dashboard to assist in tweaking mask.
			//CameraServer.getInstance().setImage(frame);
			binaryFrameToDisplay = binaryFrame;

			//filter out small particles
			float areaMin = (float)SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
			criteria[0].lower = areaMin;
			imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

			//Send particle count after filtering to dashboard
			numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Filtered particles", numParticles);

			if(numParticles > 0)
			{
				
				//Measure particles and sort by particle size
				Vector<ParticleReport> particles = new Vector<ParticleReport>();
				for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
				{
					ParticleReport par = new ParticleReport();
					par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
					par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
					par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
					par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
					particles.add(par);
		
				}
				particles.sort(null);

				//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
				//for the reader. Note that this scores and reports information about a single particle (single L shaped target). To get accurate information 
				//about the location of the tote (not just the distance) you will need to correlate two adjacent targets in order to find the true center of the tote.
				scores.Aspect = AspectScore(particles.elementAt(0));
				SmartDashboard.putNumber("Aspect", scores.Aspect);
				scores.Area = AreaScore(particles.elementAt(0));
				SmartDashboard.putNumber("Area", scores.Area);
				boolean isTote = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;

				//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
				SmartDashboard.putBoolean("IsTote", isTote);
				SmartDashboard.putNumber("Distance", computeDistance(binaryFrame, particles.elementAt(0)));
				
				//Drawing a rectangle
					//particles(0).Area
					//particles(0).BoundingRectLeft
				//NIVision.Rect(int top, int left, int height, int width);
				int top = (int)particles.elementAt(0).BoundingRectTop;
				int left = (int)particles.elementAt(0).BoundingRectLeft;
				int bottom = (int)particles.elementAt(0).BoundingRectBottom;
				int right = (int)particles.elementAt(0).BoundingRectRight;
				
				int width = right - left;
				int height = bottom - top;
				
				int r = 255;
				int g = 0;
				int b = 0;
				float color = color(r, g, b);
				
				NIVision.Rect rect = new NIVision.Rect(top, left, height, width);
				NIVision.Rect rectBig = new NIVision.Rect(top-1, left-1, height+2, width+2);
				
				NIVision.imaqDrawShapeOnImage(frame, frame, rect,
	                    DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, color);
				NIVision.imaqDrawShapeOnImage(frame, frame, rectBig,
	                    DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, color);
			} else {
				SmartDashboard.putBoolean("IsTote", false);
			}
			
			CameraServer.getInstance().setImage(frame);
				
			Timer.delay(0.005);				// wait for a motor update time
			
			 
		}*/
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
        DriveTrainSubsystem.DRIVE_SPEED = (double)SmartDashboard.getNumber("Drive Speed Max", DriveTrainSubsystem.DRIVE_SPEED);
    	CannonSubsystem.LOAD_SPEED = (double)SmartDashboard.getNumber("Cannon Load Speed", CannonSubsystem.LOAD_SPEED);
    	CannonSubsystem.LAUNCH_SPEED = (double)SmartDashboard.getNumber("Cannon Launch Speed", CannonSubsystem.LAUNCH_SPEED);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    
    /*
  //VISION
    //Comparator function for sorting particles. Returns true if particle 1 is larger
  	static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2){
  		//we want descending sort order
  		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
  	}
  	/**
 	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
 	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
 	 */
/*	double ratioToScore(double ratio) {
  		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
  	}

  	double AreaScore(ParticleReport report)
  	{
  		double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop) * (report.BoundingRectRight - report.BoundingRectLeft);
  		//Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24" of the rect.
  		return ratioToScore((49/24)*report.Area/boundingArea);
  	}
 	/**
 	 * Method to score if the aspect ratio of the particle appears to match the retro-reflective target. Target is 7"x7" so aspect should be 1
  	 */
/* 	double AspectScore(ParticleReport report)
  	{
  		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop)));
  	}
  	/**
  	 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
  	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
  	 *
  	 * @param image The image to use for measuring the particle estimated rectangle
  	 * @param report The Particle Analysis Report for the particle
  	 * @param isLong Boolean indicating if the target is believed to be the long side of a tote
  	 * @return The estimated distance to the target in feet.
  	 */
/* 	double computeDistance (Image image, ParticleReport report) {
  		double normalizedWidth, targetWidth;
  		NIVision.GetImageSizeResult size;

  		size = NIVision.imaqGetImageSize(image);
  		normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/size.width;
  		targetWidth = 7;
		return  targetWidth/(normalizedWidth*12*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
  	}
  		
  	float color(int red, int green, int blue) {
  		// each color value input is between 0-255
  		return (float)(blue*256*256 + green*256 + red);
  	}
    */
}

