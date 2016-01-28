
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.OI;
import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonEncoderMove;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class EncoderSubsystem extends Subsystem {
    
	private Encoder encoder;
	private DigitalInput inputA = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_A); //The inputs on the ROBORIO DIO
	private DigitalInput inputB = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_B);
	public final double PULSES_PER_REVOLUTION = 360 / 497;
	public final double MOVE_SPEED = 0.5;
	
	double positionTarget = 0;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public EncoderSubsystem() {
		encoder = new Encoder(inputA, inputB);
		encoder.setDistancePerPulse(PULSES_PER_REVOLUTION);

	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonEncoderMove());
    }
    
    //basic move command based on speed from joystick
    public void move(XboxController xbox){
    	double speed = -xbox.getRightStickYAxis()*MOVE_SPEED;
    	setMotor(speed);
    }
    
    //use triggers to set target position. automatically correct position
    public void moveWithTriggers(XboxController xbox){
    	double currentPosition = encoder.getRaw();
    	
    	double rTrigger = xbox.getRightTriggerAxis();
    	double lTrigger = xbox.getLeftTriggerAxis();
    	
    	if(rTrigger >= 0.05){
    		positionTarget = currentPosition + rTrigger;
    	}else if(lTrigger >= 0.05){
    		positionTarget = currentPosition - lTrigger;
    	}else{
    		positionTarget = currentPosition;
    	}
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


