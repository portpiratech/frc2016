package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.OI;
import org.usfirst.frc.team4804.robot.commands.CannonServoRotate;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ServoSubsystem extends Subsystem {
	
	public static final double BASE_ANGLE = 5; //base angle increment
	public static final double MULTIPLIER_THRESHOLD = 0.1; //value is between -1 and 1, but cannot be too close to 0
	private Servo cannonSwivel; 
	
	
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public ServoSubsystem() {
		super();
		cannonSwivel = new Servo(OI.CANNON_SWIVEL_SERVO_CHANNEL); 
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonServoRotate());
    }
    
    public void turn(XboxController xbox) {
    	turnAngle(xbox.getRightStickXAxis()); //x-axis value is a multiplier
    }
    
    public void turnAngle(double angleMultiplier) {
    	if( Math.abs(angleMultiplier) > MULTIPLIER_THRESHOLD ) {
    		double currentAngle = cannonSwivel.getAngle();		//reads value from servo
	    	double angleChange = angleMultiplier*BASE_ANGLE;	//scales value of joystick to the base angle change
	    	
	    	double newAngle = currentAngle + angleChange;			//calculates new value from old value
	    	
	    	cannonSwivel.setAngle(newAngle);
    	}
    }
	
}
