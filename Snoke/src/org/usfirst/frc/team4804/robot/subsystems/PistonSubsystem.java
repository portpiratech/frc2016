package org.usfirst.frc.team4804.robot.subsystems;

import org.usfirst.frc.team4804.robot.OI;
import org.usfirst.frc.team4804.robot.commands.CannonPistonStop;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The Piston subsystem incorporates two solenoids wired to the
 * pneumatics control module (PCM).
 */
public class PistonSubsystem extends Subsystem {
	
	private DoubleSolenoid cannonSolenoid;
	public double firingDelay = 0.5;
	
	public PistonSubsystem() {
		super();
		cannonSolenoid = new DoubleSolenoid(OI.PCM_ID, OI.SOLENOID1_PORT1, OI.SOLENOID1_PORT2); //PCM ID 1, Solenoid ports 0,1
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
		cannonSolenoid.set(Value.kForward);
	}
	
	public void retractLauncher(){
		cannonSolenoid.set(Value.kReverse);
	}
	
	public void stopLauncher() {
		cannonSolenoid.set(Value.kOff);
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
