
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
    
	private Encoder encoder;
	private DigitalInput inputA = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_A); //The inputs on the RoboRIO DIO
	private DigitalInput inputB = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_B);
	public final double DEGREES_PER_PULSE = 360 / 497;
	public final double TRIGGER_TOLERANCE = 0.05;
	public final double POSITION_TOLERANCE = 1;
	public final double POSITION_MAX = 90;
	public final double SPEED_TOLERANCE = 0.01;
	public final double SPEED_MAX = 0.3;
	
	double positionTarget = 0;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public EncoderSubsystem() {
		encoder = new Encoder(inputA, inputB);
		encoder.setDistancePerPulse(DEGREES_PER_PULSE);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonEncoderMove());
    }
    
    //basic move command based on speed from joystick
    public void move(XboxController xbox){
    	double speed = -xbox.getRightStickYAxis()*SPEED_MAX;
    	setMotor(speed);
    }
    
    //use triggers to set target position, automatically correct speed (see last year's code for template)
    public void moveWithTriggers(XboxController xbox){
    	double currentPosition = encoder.getRaw();
    	double currentSpeed = encoder.getRate();
    	
    	int dpad = xbox.getDPad();
    	double rTrigger = xbox.getRightTriggerAxis();
    	double lTrigger = xbox.getLeftTriggerAxis();
    	
    //find target position
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
	    	if(rTrigger >= TRIGGER_TOLERANCE && positionTarget <= POSITION_MAX){
	    		positionTarget = currentPosition + rTrigger;
	    	}else if(lTrigger >= TRIGGER_TOLERANCE && positionTarget >= 0){
	    		positionTarget = currentPosition - lTrigger;
	    	}else{
	    		positionTarget = currentPosition;
	    	}
    	}
    	
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
    		setMotor(Math.signum(finalSpeed)*SPEED_MAX);
    	}else{
    		setMotor(finalSpeed);
    	}
    	
    	Timer.delay(0.005);
    	
    }
    
    
    //LAST YEAR'S CODE
    
    /*public void moveTowardTargetPosition() {
    	double currentSpeed = 1;
    	double finalSpeed = 0;
    	double posError = 10;
    	SmartDashboard.putString("Encoder Command", "Lock Speed Mode");

    	currentSpeed = getMotorSpeed();
    	posError = getTargetPosition() - encoder.getRaw();
        if (posError != 0.0) {
	    	currentSpeed = getMotorSpeed();
	    	posError = getTargetPosition() - encoder.getRaw();
	    	SmartDashboard.putNumber("Position Error", posError);
	    		    	
	    	if(posError >= posTolerance) {
	    		// increase speed toward floor if current position is lower than target (too close to the robot)
	    		SmartDashboard.putString("Lock Speed Command", "Incrementing Speed");
	    		finalSpeed = (currentSpeed + incGain*Math.abs(posError));
	    		finalSpeed = (finalSpeed * Math.abs(posError/(maxSpeed*100)));
	    	}
  
	    	if(posError <= -posTolerance) {
	    		// increase speed toward robot if current position is greater than target (too close to the floor)
	    		SmartDashboard.putString("Lock Speed Command", "Decrementing Speed");
	    		finalSpeed = (currentSpeed - incGain*Math.abs(posError));
	    		finalSpeed = (finalSpeed * Math.abs(posError/(maxSpeed*100)));

	    	}
	    	
	    	if(Math.abs(posError) < posTolerance) {
	    		// do nothing if the current position is within reasonable bounds
	    		SmartDashboard.putString("Lock Speed Command", "Doing Nothing");
	    		finalSpeed = currentSpeed;
	    	}
	    	
	    	SmartDashboard.putString("Lock Speed Command", "Setting Speed");
	    	if(Math.abs(finalSpeed) > maxSpeed) {
	    		int sign = (int) (Math.abs(finalSpeed)/finalSpeed);
	    		setMotorSpeed(sign*maxSpeed);
	    	}else{
	    		setMotorSpeed(finalSpeed);
	    	}
	    	
	    	//Timer.delay(0.005);
	    	
	    	SmartDashboard.putNumber("Final Speed", finalSpeed);
	    	SmartDashboard.putNumber("Encoder Position Actual", readEncoder());
	    	SmartDashboard.putNumber("Encoder Position Target", getTargetPosition());
	    	SmartDashboard.putString("Lock Speed Command", "Looping");
    	}
    	SmartDashboard.putString("Lock Speed Command", "idle");
    	return;
    }*/

	private void setMotor(double speed) {
		Robot.cannonEncoderMotor.set(speed);
	}

	private double getMotorSpeed() {
		return encoder.getRate(); //we still need to set the distance per pulse
	}
}


