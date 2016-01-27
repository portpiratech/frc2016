
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.OI;
import org.usfirst.frc.team4804.robot.commands.ExampleCommand;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class EncoderSubsystem extends Subsystem {
    
	private Encoder encoder;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public EncoderSubsystem() {
		//encoder = new Encoder(OI.CANNON_TILT_MOTOR_ID); 			DOESN'T WORK
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new ExampleCommand());
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
    }

	private void setMotorSpeed(double speed) {
		encoder.set(speed);
	}

	private double getMotorSpeed() {
		return encoder.getSpeed();
	}*/
}


