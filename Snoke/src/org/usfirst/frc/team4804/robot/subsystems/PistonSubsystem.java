package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.Robot;
import org.usfirst.frc.team4804.robot.commands.CannonPistonStop;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The Piston subsystem incorporates two solenoids wired to the
 * pneumatics control module (PCM).
 */
public class PistonSubsystem extends Subsystem {

	public PistonSubsystem() {
		super();
		//solenoid2 = new DoubleSolenoid(OI.SOLENOID2_PORT1,OI.SOLENOID2_PORT2); move to Robot.java when we have a use for this
	}


	public void initDefaultCommand() {
		//setDefaultCommand(new *);
		setDefaultCommand(new CannonPistonStop());
	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	public void log() {
	}
	
	// Cannon solenoid (Launcher)
	public void extendLauncher() {
		Robot.cannonSolenoid.set(Value.kForward);
	}
	
	public void retractLauncher(){
		Robot.cannonSolenoid.set(Value.kReverse);
	}
	
	public void stopLauncher() {
		Robot.cannonSolenoid.set(Value.kOff);
	}
	
	/* last year's code
	public void extendArms() {
		solenoid2.set(Value.kForward);
	}
	
	public void retractArms() {
		solenoid2.set(Value.kReverse);
	}
	
	public void stopArms() {
		solenoid2.set(Value.kOff);
	}
	*/
}
