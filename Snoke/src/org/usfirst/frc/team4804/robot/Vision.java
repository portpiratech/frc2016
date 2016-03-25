package org.usfirst.frc.team4804.robot;

import java.util.Comparator;
import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class Vision {
	
	 //A structure to hold measurements of a particle
  	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport> {
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
  	}
  	
   //Structure to represent the scores for the various tests used for target identification
  	public class Scores {
  		double Area;
  		double Aspect;
  	};

   //Images
  	Image frame;
  	public Image binaryFrame;
  	Image binaryFrameToDisplay;
  	int imaqError;
  	int session;
  	USBCamera targetCam;
  	CameraServer server;
	
   //Constants
  	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(40, 180);	//Default hue range for green-cyan LED
  	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(100, 255);	//Default saturation range for green-cyan LED
  	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(200, 255);	//Default value range for green-cyan LED
  	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
  	double LONG_RATIO = 2.22; //(Target long side = 26.9 / Target height = 12.1) = 2.22
  	double SHORT_RATIO = 1.4; //(Target short side = 16.9 / Target height = 12.1) = 1.4
  	double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
  	double VIEW_ANGLE = 48.5; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 68.5 for HD3000 640x480
  	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
  	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
  	Scores scores = new Scores();
    
   //Vision
    //CameraServer server;
    //int session;
    //Image frame;
  	
  	public double errorAimingX;
  	public double distanceFeet;
  	long lastFrameProcessTimeMs;
	long captureIntervalMs;
  	
  //methods
  	
  	/**
  	 * Initializes vision system
  	 */
  	public Vision() {
		// create images
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
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
		 			
//		session = NIVision.IMAQdxOpenCamera("cam0",
//		          NIVision.IMAQdxCameraControlMode.CameraControlModeController);
//		NIVision.IMAQdxConfigureGrab(session);
		 	
//		// old SimpleVision stuff
//	    frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
//	    // the camera name (ex "cam0") can be found through the roborio web interface
//	    session = NIVision.IMAQdxOpenCamera("cam0",
//	             NIVision.IMAQdxCameraControlMode.CameraControlModeController);
//	    NIVision.IMAQdxConfigureGrab(session);
		
		lastFrameProcessTimeMs = 0;
		captureIntervalMs = 100;
		
		targetCam = new USBCamera("cam0");
    	targetCam.setBrightness(0);
    	targetCam.setExposureManual(0);
    	targetCam.setExposureHoldCurrent();
//    	
    	targetCam.updateSettings();
//    	
    	SmartDashboard.putNumber("Brightness", 0); //targetCam.getBrightness());
    	SmartDashboard.putBoolean("Update Camera", false);
    	
    	targetCam.openCamera();
    	targetCam.startCapture();
    	
    	server = CameraServer.getInstance();
//    	server.setQuality(50);
	}
  	
  	/**
  	 * Displays the camera frame with no processing
  	 */
  	public void frameAutoDisplay() {
  		//NIVision.IMAQdxGrab(session, frame, 1);
  		//CameraServer.getInstance().setImage(frame);
  		
  		//if (SmartDashboard.getBoolean("Update Camera")) {
//          // robot code here!
	            targetCam.setExposureManual(0);
	            targetCam.setBrightness((int) SmartDashboard.getNumber("Brightness"));
	        	targetCam.setExposureHoldCurrent();
	            targetCam.updateSettings();
				SmartDashboard.putBoolean("Update Camera", false);
      	//}
      	targetCam.getImage(frame);
      	rotate180(frame);
      	CameraServer.getInstance().setImage(frame);
  	}
  	
  	/**
	 * Processes the camera frame
	 */
	public void frameProcess() {
		double distanceFeetTemp;
		//Image savedFrame;
		
		if(System.currentTimeMillis() - lastFrameProcessTimeMs < captureIntervalMs) {
			return;
		}

		lastFrameProcessTimeMs = System.currentTimeMillis();
		
		//read file in from disk. For this example to run you need to copy image.jpg from the SampleImages folder to the
		//directory shown below using FTP or SFTP: http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
		
		//NIVision.imaqReadFile(frame, "/home/lvuser/SampleImages/image.jpg");
		//NIVision.IMAQdxGrab(session, frame, 1);
		
		//if (SmartDashboard.getBoolean("Update Camera")) {
//          // robot code here!
	            targetCam.setExposureManual(0);
	            targetCam.setBrightness((int) SmartDashboard.getNumber("Brightness"));
	        	targetCam.setExposureHoldCurrent();
	        	//targetCam.getImage(frame);
	        	//savedFrame = frame;
	            targetCam.updateSettings();
				SmartDashboard.putBoolean("Update Camera", false);
      	//}
		targetCam.getImage(frame);
      	
      	rotate180(frame);

		//Update threshold values from SmartDashboard. For performance reasons it is recommended to remove this after calibration is finished.
		TOTE_HUE_RANGE.minValue = (int)SmartDashboard.getNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
		TOTE_HUE_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
		TOTE_SAT_RANGE.minValue = (int)SmartDashboard.getNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
		TOTE_SAT_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
		TOTE_VAL_RANGE.minValue = (int)SmartDashboard.getNumber("Tote val min", TOTE_VAL_RANGE.minValue);
		TOTE_VAL_RANGE.maxValue = (int)SmartDashboard.getNumber("Tote val max", TOTE_VAL_RANGE.maxValue);

		//Threshold the image looking for threshold values
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);

		//Send particle count to dashboard
		int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		//SmartDashboard.putNumber("Masked particles", numParticles);

		//binaryFrameToDisplay = binaryFrame;

		//filter out small particles
		float areaMin = (float)SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
		criteria[0].lower = areaMin;
		imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);


		//Send masked image to dashboard to assist in tweaking mask.
		//CameraServer.getInstance().setImage(binaryFrame);
		
		//Send particle count after filtering to dashboard
		numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		//SmartDashboard.putNumber("Filtered particles", numParticles);

		if(numParticles >= 1) {
			
			//Measure particles and sort by particle size
			Vector<ParticleReport> particles = new Vector<ParticleReport>();
			double largestArea = 0;
			int largestAreaIndex = 0;
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
				ParticleReport par = new ParticleReport();
				par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
				par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);

//				par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
//				par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
//				par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
//				par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
				particles.add(par);
				if(par.Area > largestArea) {
					largestArea = par.Area;
					largestAreaIndex = particleIndex;
				}
			}
			//particles.sort(null);
			
			//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
			//for the reader. Note that this scores and reports information about a single particle (single L shaped target). To get accurate information 
			//about the location of the tote (not just the distance) you will need to correlate two adjacent targets in order to find the true center of the tote.
			ParticleReport largestParticle = particles.elementAt(largestAreaIndex);
			largestParticle.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, 0, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
			largestParticle.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, 0, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
			largestParticle.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, 0, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
			largestParticle.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, 0, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
			
			scores.Aspect = AspectScore(largestParticle);
			//SmartDashboard.putNumber("Aspect", scores.Aspect);
			scores.Area = AreaScore(largestParticle);
			//SmartDashboard.putNumber("Area", scores.Area);
			boolean isTote = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;

			//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
			SmartDashboard.putBoolean("IsTote", isTote);
			distanceFeetTemp = computeDistance(binaryFrame, largestParticle);
			SmartDashboard.putNumber("Distance (Temp)", distanceFeetTemp);
			SmartDashboard.putNumber("Distance (in) (Temp)", distanceFeetTemp * 12.0);
			SmartDashboard.putNumber("Launch Angle", launchAngle(distanceFeetTemp)); //1 ft = 0.3048 m
			//SmartDashboard.putString("Launch Angle (Test)", "test1");
			errorAimingX = computeErrorAimingX(binaryFrame, largestParticle);
			
			//Bounding rectangle params
			int top = (int)largestParticle.BoundingRectTop;
			int left = (int)largestParticle.BoundingRectLeft;
			int bottom = (int)largestParticle.BoundingRectBottom;
			int right = (int)largestParticle.BoundingRectRight;
			int width = right - left;
			int height = bottom - top;
			
			//Pick a colour
			int r = 255;
			int g = 0;
			int b = 0;
			float color = color(r, g, b);
			
			drawRectangle(top, left, width, height, color, 4, frame);
		} else {
			SmartDashboard.putBoolean("IsTote", false);
			distanceFeetTemp = distanceFeet;
			
			SmartDashboard.putNumber("Distance (Temp)", distanceFeetTemp);
			SmartDashboard.putNumber("Distance (in) (Temp)", distanceFeetTemp * 12.0);
			SmartDashboard.putNumber("Launch Angle", launchAngle(distanceFeetTemp)); //1 ft = 0.3048 m
			//SmartDashboard.putString("Launch Angle (Test)", "test2");
			
			/*distanceFeet = 0;
			SmartDashboard.putNumber("Distance", distanceFeet);
			SmartDashboard.putNumber("Distance (in)", distanceFeet * 12.0);
			SmartDashboard.putNumber("Launch Angle", launchAngle(distanceFeet*0.3048)); //1 ft = 0.3048 m
			errorAimingX = 0;*/
		}
		
		CameraServer.getInstance().setImage(frame);
		distanceFeet = distanceFeetTemp;
		SmartDashboard.putNumber("Distance", distanceFeet);
		SmartDashboard.putNumber("Distance (in)", distanceFeet * 12.0);
		SmartDashboard.putNumber("Launch Angle", launchAngle(distanceFeet*0.3048)); //1 ft = 0.3048 m
		
		//Timer.delay(0.1);				// wait for a motor update time
	}
	
	/**
	 * @param top Top bound of rectangle
	 * @param left Left bound of rectangle
	 * @param width Width of rectangle
	 * @param height Height of rectangle
	 * @param color RGB color value
	 * @param thickness Border thickness in pixels (use values >1 for best results)
	 * @param image Image to draw on
	 */
	private void drawRectangle(int top, int left, int width, int height, float color, int thickness, Image image) {
		for(int i=1; i<=thickness; i++){
			//define bounds
			NIVision.Rect rect = new NIVision.Rect(top, left, height, width);
			
			//draw on image
			NIVision.imaqDrawShapeOnImage(frame, frame, rect,
			        DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, color);
			
			//set up bounds for next largest rectangle
			top--;
			left--;
			height += 2;
			width += 2;
		}
	}
	
    //Comparator function for sorting particles. Returns true if particle 1 is larger
  	static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2) {
  		//we want descending sort order
  		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
  	}
  	/**
 	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
 	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
 	 */
	double ratioToScore(double ratio) {
  		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
  	}

  	double AreaScore(ParticleReport report) {
  		double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop) * (report.BoundingRectRight - report.BoundingRectLeft);
  		//Tape is 20" bottom edge + 7" side edges so 140" bounding rect. With 2" wide tape it covers 50" of the rect.
  		return ratioToScore((140.0/50.0)*report.Area/boundingArea);
  	}
 	/**
 	 * Method to score if the aspect ratio of the particle appears to match the retro-reflective target. Target is 20"x14" so aspect should be 10/7=1.429
  	 */
 	double AspectScore(ParticleReport report) {
  		return ratioToScore((7.0/10.0)*(report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop));
  	}
 	
 	/**
  	 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
  	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
  	 *
  	 * @param image The image to use for measuring the particle estimated rectangle
  	 * @param report The Particle Analysis Report for the particle
  	 * @return The estimated distance to the target in feet.
  	 */
 	public double computeDistance (Image image, ParticleReport report) {
  		double targetWidthInches, targetWidthFeet, targetWidthPixels, imageWidthPixels;
  		NIVision.GetImageSizeResult size = NIVision.imaqGetImageSize(image);
  		
  		double leftPixels = report.BoundingRectLeft;
  		double rightPixels = report.BoundingRectRight;
  		//double topPixels = report.BoundingRectTop;
  		//double bottomPixels = report.BoundingRectBottom;
  		
  		imageWidthPixels = size.width;
  		targetWidthPixels = rightPixels - leftPixels; //units are pixels
  		targetWidthInches = 20; //inches
  		targetWidthFeet = targetWidthInches/12.0;	//units are feet. target is 20 inches wide, or 20/12 feet wide
  		
  		SmartDashboard.putNumber("PPI", targetWidthPixels / 20.0);
//  		SmartDashboard.putString("computeDistance", targetWidthFeet + "*" + imageWidthPixels + "/(" + targetWidthPixels + "*2*tan(" + VIEW_ANGLE + " * 3.14159/(180*2)))");
  		
  		return targetWidthFeet*imageWidthPixels/(targetWidthPixels*2*Math.tan(VIEW_ANGLE*Math.PI/(180*2))); //Math.tan() takes angle in radians
  	}
 	
 	double computeErrorAimingX (Image image, ParticleReport report) {
 		double cameraOffset = -0.05;
 		
  		NIVision.GetImageSizeResult size = NIVision.imaqGetImageSize(image);
  		double imageWidthPixels = size.width;
  		
  		double leftPixels = report.BoundingRectLeft;
  		double rightPixels = report.BoundingRectRight;
  		
  		double centerPixels = (leftPixels+rightPixels)/2.0;
  		
  		//Axy = (Pxy - res/2)/(res/2)
  		double leftAiming = (leftPixels - imageWidthPixels/2.0)/(imageWidthPixels/2.0);
  		double rightAiming = (rightPixels - imageWidthPixels/2.0)/(imageWidthPixels/2.0);
  		
  		double centerAiming = (leftAiming+rightAiming)/2.0 + cameraOffset;
  		
  		SmartDashboard.putNumber("centerPixels", centerPixels);
  		SmartDashboard.putNumber("centerAiming", centerAiming);
  		
  		return centerAiming;
 	}
 	
 	//calculate the angle the encoder should be
    public double launchAngle(double distanceFt) {
    	final double g = 9.81; 	//acceleration due to gravity. (m/s^2)
    	double distanceM = distanceFt/3.281; //distance in meters
    	
       //constants
    	double v = 8.686; 		//initial velocity. (m/s)	Roughly 14 mph = 6.26 m/s
    	double heightM = 2.311; 		//height of target. (m)
    	
       //optimum launch angle so that ball passes through target at peak of trajectory
    	/*SmartDashboard.putString("Square Root Argument", Double.toString(Math.pow(v, 4) - g*(g*Math.pow(distanceM,2) + 2*heightM*Math.pow(v,2)) ));
    	SmartDashboard.putString("Denominator Argument", Double.toString(g*distanceM));
    	SmartDashboard.putString("Arctangent Argument", Double.toString((Math.pow(v, 2) - Math.sqrt( Math.pow(v, 4) - g*(g*Math.pow(distanceM,2) + 2*heightM*Math.pow(v,2)) ))/(g*distanceM)));
    	SmartDashboard.putString("Arctangent Result", Double.toString(Math.atan( (Math.pow(v, 2) - Math.sqrt( Math.pow(v, 4) - g*(g*Math.pow(distanceM,2) + 2*heightM*Math.pow(v,2)) ))/(g*distanceM) )));*/
    	double launchAngle = Math.atan( (Math.pow(v, 2) - Math.sqrt( Math.pow(v, 4) - g*(g*Math.pow(distanceM,2) + 2*heightM*Math.pow(v,2)) ))/(g*distanceM) );
    	return launchAngle*180.0/Math.PI;
    }
  		
  	float color(int red, int green, int blue) {
  		// each color value input is between 0-255
  		return (float)(blue*256*256 + green*256 + red);
  	}
  	
  	public void rotate180(Image frame) {
  		NIVision.imaqFlip(frame, frame, NIVision.FlipAxis.HORIZONTAL_AXIS);
  		NIVision.imaqFlip(frame, frame, NIVision.FlipAxis.VERTICAL_AXIS);
  	}
}
