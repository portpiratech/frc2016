
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.OI;
import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonEncoderMove;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class EncoderSubsystem extends Subsystem {
    
   //objects
	private Encoder encoder;
	private DigitalInput inputA = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_A); //The inputs on the RoboRIO DIO
	private DigitalInput inputB = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_B);
	
   //constants
	public final double DEGREES_PER_PULSE = 360 / 497;
	public final double TRIGGER_TOLERANCE = 0.05;
	public final double POSITION_TOLERANCE = 1;
	public final double POSITION_MAX = 90;
	public final double SPEED_TOLERANCE = 0.01;
	public final double SPEED_MAX = 0.3;
	
	double positionTarget = 0;
	
    // Constructor
	public EncoderSubsystem() {
		encoder = new Encoder(inputA, inputB);
		encoder.setDistancePerPulse(DEGREES_PER_PULSE);
	}
	// Default command
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonEncoderMove());
    }
    
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
  //Basic Encoder Motor speed/position methods
    private void setMotorSpeed(double speed) {
		Robot.cannonEncoderMotor.set(speed);
	}

	private double getMotorSpeed() {
		return encoder.getRate(); //we still need to set the distance per pulse
	}
	
	private void resetMotor() {
		Robot.cannonEncoderMotor.reset();
	}
    
  //Position calculation/movement/math methods
   //basic move commands
    public void moveManual(XboxController xbox){
    	//double speed = -xbox.getRightStickYAxis()*SPEED_MAX;
    	//setMotorSpeed(speed);
    	targetPositionWithTriggers(xbox);
    	move();
    }
    
    public void moveAuto(double distance){ //feed "distance" from an analog input
    	positionTarget = launchAngle(distance);
    	move();
    }
    
   //use triggers to set target position, automatically correct speed (see last year's code for template)
    public void targetPositionWithTriggers(XboxController xbox){
       //read encoder position
    	double currentPosition = encoder.getRaw();
    	
       //read xbox controller
    	int dpad = xbox.getDPad();
    	double rTrigger = xbox.getRightTriggerAxis();
    	double lTrigger = xbox.getLeftTriggerAxis();
    	
       //calculate target position
    	switch(dpad){
    	
    	case 0:
    		//dpad up
    		if(currentPosition<=POSITION_MAX) positionTarget = Math.round(currentPosition/10.0)+10.0;
    		break;
    		
    	case 4:
    		//dpad down
    		if(currentPosition>=10.0) positionTarget = Math.round(currentPosition/10.0)-10.0;
    		break;
    	
    	default:
    		//dpad not pressed, calculate from triggers
	    	if(rTrigger >= TRIGGER_TOLERANCE && positionTarget <= POSITION_MAX){
	    		positionTarget = currentPosition + rTrigger;
	    	}else if(lTrigger >= TRIGGER_TOLERANCE && positionTarget >= 0){
	    		positionTarget = currentPosition - lTrigger;
	    	}else{
	    		positionTarget = currentPosition;
	    	}
	    	break;
    	}
    }
    
   //move and regulate position
    public void move() {
    	double currentPosition = encoder.getRaw();
    	double currentSpeed = encoder.getRate();
    	
       //check position & calculate speed
    	double positionError = currentPosition - positionTarget;
    	double finalSpeed = 0;
    	
    	if(positionError >= POSITION_TOLERANCE) {
    		// increase speed toward floor if current position is too high up
    		SmartDashboard.putString("Lock Speed Command", "Accelerating toward floor");
    		finalSpeed = currentSpeed + Math.abs(positionError);
    		finalSpeed = finalSpeed * Math.abs(positionError/SPEED_MAX);
    	}
    	if(positionError <= -POSITION_TOLERANCE) {
    		// increase speed away from floor if current position is too low
    		SmartDashboard.putString("Lock Speed Command", "Accelerating away from floor");
    		finalSpeed = currentSpeed - Math.abs(positionError);
    		finalSpeed = finalSpeed * Math.abs(positionError/SPEED_MAX);
    	}
    	if(Math.abs(positionError) < POSITION_TOLERANCE) {
    		// do nothing if the current position is within reasonable bounds of target
    		finalSpeed = currentSpeed;
    		if (finalSpeed >= 0) SmartDashboard.putString("Lock Speed Command", "Moving constantly toward floor");
    		if (finalSpeed < 0) SmartDashboard.putString("Lock Speed Command", "Moving constantly away from floor");
    	}
    	
       //set final speed
    	if(Math.abs(finalSpeed) > SPEED_MAX) {
    		//make sure speed isn't out of bounds
    		setMotorSpeed(Math.signum(finalSpeed)*SPEED_MAX);
    	}else{
    		setMotorSpeed(finalSpeed);
    	}
    	
    	Timer.delay(0.005);
    }
    
   //calculate the angle the encoder should be
    public double launchAngle(double distance) {
    	final double g = 9.81; 	//acceleration due to gravity. (m/s^2)
    	
       //constants
    	double v = 6; 			//initial velocity. need to test more to calculate. (m/s)
    	double height = 2; 		//height of target. (m) might need to make variable based on angle?
    	
       //optimum launch angle so that ball passes through target at peak of trajectory
    	double numerator = pow(v, 2) + sqrt( pow(v, 4) - g*(g*pow(distance,2) + 2*height*pow(v,2)) );
    	double denominator = g*distance;
    	double launchAngle = atan(numerator/denominator);
    	return launchAngle;
    }
    
   //convenient math methods (see launchAngle())
    double atan(double x) { return Math.atan(x); }
    double pow(double x, double y) { return Math.pow(x, y); }
    double sqrt(double x) { return Math.sqrt(x); }
}


