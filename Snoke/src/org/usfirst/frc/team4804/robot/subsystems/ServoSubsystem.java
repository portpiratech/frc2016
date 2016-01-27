package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.OI;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ServoSubsystem extends Subsystem {
	
	private Servo cannonSwivel; 
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	

	public ServoSubsystem() {
		super();
		// create servo object
		cannonSwivel = new Servo(OI.CANNON_SWIVEL_SERVO_CHANNEL); 
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new ExampleCommand());
    }
    
    public void turn(double angle) {
    	cannonSwivel.setAngle(angle);
    }
    
    public void turnRight() {
		
	}
	
	public void turnLeft() {
		
	}
	
}
