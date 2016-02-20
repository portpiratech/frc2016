
package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonEncoderMove;

import com.portpiratech.xbox360.XboxController;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class EncoderSubsystem extends PIDSubsystem {
    
   //objects
	/*private Encoder encoder;
	private DigitalInput inputA = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_A); //The inputs on the RoboRIO DIO
	private DigitalInput inputB = new DigitalInput(OI.CANNON_ENCODER_CHANNEL_B);*/
	
   //constants
	public final double DEGREES_PER_PULSE = 360 / 497;
	public final double TRIGGER_TOLERANCE = 0.05;
	public final double POSITION_TOLERANCE = 10;
	public final double POSITION_MAX = 1000;
	public final double SPEED_TOLERANCE = 0.03;
	public final double SPEED_MAX = 1;
	
	double angleOffset = 45; //degrees; need to measure this value
	double positionTarget = 0;
	boolean calibrated = false;
	
    // Constructor
	public EncoderSubsystem() {
		super(0.1, 0.0, 0.0);
		this.disable();
		/*encoder = new Encoder(inputA, inputB);
		encoder.setDistancePerPulse(DEGREES_PER_PULSE);*/
	}
	// Default command
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new CannonEncoderMove());
    	setDefaultCommand(new CannonEncoderMove());
    }
    
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
  //Basic Encoder Motor speed/position methods
    private void setMotor(double speed) {
    	if (Math.abs(speed) < SPEED_TOLERANCE) speed = 0;
		Robot.cannonEncoderMotor.set(speed);
		positionTarget = Robot.cannonEncoderMotor.get();
	}
    
    public void move(XboxController xbox) {
    	setMotor(xbox.getLeftStickYAxis()*0.4);
    }

	/*private double getMotorSpeed() {
		return encoder.getRate(); //we still need to set the distance per pulse <--(I think we already did?)
	}*/
	
	public void resetMotor() {
		//hit first lim
		setMotor(-0.1);
		while(!Robot.cannonEncoderMotor.isFwdLimitSwitchClosed()) {}
		setMotor(0);
		double angle1 = Robot.cannonEncoderMotor.getEncPosition();
		
		//move to second lim
		setMotor(0.1);
		while(!Robot.cannonEncoderMotor.isRevLimitSwitchClosed()) {}
		setMotor(0);
		
		//measure angle
		double angle2 = Robot.cannonEncoderMotor.getEncPosition();
		
		//move until centered
		setMotor(-0.1);
		while(Robot.cannonEncoderMotor.get() > (angle2-angle1)/2) {}
		setMotor(0);
	}
    
  //Position calculation/movement/math methods
   //basic move commands
    /*public void moveManual(XboxController xbox){
    	//double speed = -xbox.getLeftStickYAxis()*SPEED_MAX;
    	//setMotor(speed);
    	targetPositionWithTriggers(xbox);
    	move2();
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
    	
       //calculate target position             Tommy: Let's not set the position target equal to any variant of its current position
    	switch(dpad){
    	
    	case 0:
    		//dpad up
    		if(currentPosition<=POSITION_MAX) //positionTarget = Math.floor(currentPosition/10.0)+10.0;
    			positionTarget += 10;
    		break;
    		
    	case 4:
    		//dpad down
    		if(currentPosition>=10.0) //positionTarget = Math.ceil(currentPosition/10.0)-10.0;
    			positionTarget -= 10;
    		break;
    	
    	/*default:
    		//dpad not pressed, calculate from triggers
	    	if(rTrigger >= TRIGGER_TOLERANCE && positionTarget <= POSITION_MAX){
	    		positionTarget = currentPosition + rTrigger;
	    	}else if(lTrigger >= TRIGGER_TOLERANCE && positionTarget >= 0){
	    		positionTarget = currentPosition - lTrigger;
	    	}else{
	    		//positionTarget = currentPosition;
	    	}
	    	break;*//*
    	}
    	
    	if (positionTarget > POSITION_MAX) positionTarget = POSITION_MAX;
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
    		setMotor(Math.signum(finalSpeed)*SPEED_MAX);
    	}else{
    		setMotor(finalSpeed);
    	}
    	
    	Timer.delay(0.010);	//test different values
    }
    
    public void move2(){
    	double currentPosition = encoder.getRaw();
    	double currentSpeed = encoder.getRate();
    	
    	double positionError = currentPosition - positionTarget;
    	double speed;
    	if (Math.abs(positionError) >= 200){ //sets the speed to travel faster toward the target position when further away
    		speed = SPEED_MAX;
    	}else if (Math.abs(positionError) >= 100){
    		speed = SPEED_MAX*.6;
    	}else if (Math.abs(positionError) >= 50){
    		speed = SPEED_MAX*.3;
    	}else{
    		speed = SPEED_MAX*.1;
    	}
    	SmartDashboard.putNumber("Position Error", positionError);
    	SmartDashboard.putNumber("Speed: ", speed);
    	
    	if (currentPosition > positionTarget && Math.abs(positionError) > POSITION_TOLERANCE){
    		setMotor(speed);
    		SmartDashboard.putString("Motor set: ", "negative");
    	}else if (currentPosition < positionTarget && Math.abs(positionError) > POSITION_TOLERANCE){
    		setMotor(-speed);
    		SmartDashboard.putString("Motor set: ", "positive");
    	}else{ //will only run this if the current position = the target or the error < the tolerance
    		setMotor(0);
    		SmartDashboard.putString("Motor set: ", "nothing");
    	}
    	
    	SmartDashboard.putNumber("CURRENT POSITION", currentPosition);
    	SmartDashboard.putNumber("TARGET POSITION",  positionTarget);
    	
    	Timer.delay(0.010);
    }*/
    
   //PID methods
	@Override
	protected double returnPIDInput() {
		//double distance = Robot.vision.distanceFeet;
		//double distanceX = distance * Math.cos(Robot.cannonEncoderMotor.get()*DEGREES_PER_PULSE - angleOffset);
		//double targetAngle = Robot.vision.launchAngle(distanceX) + angleOffset;
		//return targetAngle - Robot.cannonEncoderMotor.get()*DEGREES_PER_PULSE; //error
		return positionTarget+angleOffset-Robot.cannonEncoderMotor.get()*DEGREES_PER_PULSE;
	}
	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		
	}
}


