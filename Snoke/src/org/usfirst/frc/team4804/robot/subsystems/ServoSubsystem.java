package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.OI;
import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonServoRotate;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ServoSubsystem extends Subsystem {
	
	public static final double BASE_ANGLE = 1; //base angle increment
	public static final double MAX_ANGLE = 30; //in either direction. might only be positive later
	public static final double JOYSTICK_TOLERANCE = 0.1; //value is between -1 and 1, but cannot be too close to 0
	private Servo cannonSwivel; 
	
	
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public ServoSubsystem() {
		super();
		cannonSwivel = new Servo(OI.CANNON_SWIVEL_SERVO_CHANNEL); 
		cannonSwivel.setAngle(0); //resets servo. probably temporary.
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CannonServoRotate());
    }
    
    public void turn(XboxController xbox) {
    	turnAngle(xbox.getLeftStickXAxis(), xbox.getDPad()); //x-axis value is a multiplier
    }
    
    public void turnAngle(double angleMultiplier, int dpad) {
    	double currentAngle = cannonSwivel.getAngle();		//reads value from servo
    	double angleChange = 0;
    	double newAngle = 0;
    	
    	switch(dpad) {
    	
    	case -1:
	    	//controller axis always returns insignificant values, fixes creeping
	    	if( Math.abs(angleMultiplier) > JOYSTICK_TOLERANCE ) {
	    		
	    		currentAngle = cannonSwivel.getAngle();		//reads value from servo
	    		angleChange = angleMultiplier*BASE_ANGLE;	//scales value of joystick to the base angle change
	    		
	    		// if already at or above max angle, and trying to move further in that direction, don't do anything.
	    		if(Math.abs(currentAngle) >= MAX_ANGLE && Math.signum(currentAngle) == Math.signum(angleChange)) {
	    			// keep angle constant.
	    			newAngle = currentAngle;
	    		}else{
	    			// otherwise, update the angle.
	    			newAngle = currentAngle + angleChange;
	    		}
	    	}
	    	break;
	    	
    	case 2:
    		//right
    		//if(currentAngle<=MAX_ANGLE) positionTarget = Math.round(currentPosition/10.0)+10.0;
    		break;
    		
    	case 6:
    		//left
    		
    		break;
    	}
    	cannonSwivel.setAngle(newAngle);
    	SmartDashboard.putNumber("Cannon Swivel Angle:", cannonSwivel.getAngle());
    }
	
}
