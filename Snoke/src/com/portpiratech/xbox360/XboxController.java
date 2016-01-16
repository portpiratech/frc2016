package com.portpiratech.xbox360;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Class which adds helper methods to better use an XBox Controller without having
 * to do so much configuration within a FRC 2015 Robot.
 * @author PortPiratech
 *
 */
public class XboxController extends Joystick {

	/**
	 * Constructor which takes the XBox Controller port and calls super class.
	 * 
	 * @param port that Xbox Controller is configured on.
	 */
	public XboxController(int port) {
		super(port);
		// TODO Auto-generated constructor stub
	}
	
	/**
	 * 
	 * @return a double value representing the position of the Left Stick on the XBox Controller.
	 */
	public double getLeftStickYAxis() {
		return getY();
	}
	
	/**
	 * 
	 * @return a double value representing the position of the Right Stick on the XBox Controller.
	 */
	public double getRightStickYAxis() {
		return getRawAxis(5);
	}
	
	/**
	 * This method returns the A Button from the XBox Controller
	 * @return
	 */
	public JoystickButton getAButton() {
		return new JoystickButton(this,1);
	}
	
	/**
	 * This method returns the B Button from the XBox Controller
	 * @return JoystickButton Mapped to Button B on Xbox Controller
	 */
	public JoystickButton getBButton() {
		return new JoystickButton(this, 2);
	}
	
	/**
	 * This method returns the X Button from the XBox Controller
	 * @return JoystickButton Mapped to Button X on Xbox Controller
	 */
	public JoystickButton getXButton() {
		return new JoystickButton(this, 3);
	}
	
	/**
	 * This method returns the Y Button from the XBox Controller
	 * @return JoystickButton Mapped to Button Y on Xbox Controller
	 */
	public JoystickButton getYButton() {
		return new JoystickButton(this, 4);
	}
	
	/**
	 * This method returns the left bumper button from the XBox Controller
	 * @return JoystickButton Mapped to button left bumper on Xbox Controller
	 */
	public JoystickButton getLeftBumper() {
		return new JoystickButton(this, 5);
	}
	
	/**
	 * This method returns the right bumper button from the XBox Controller
	 * @return JoystickButton Mapped to button right bumper on Xbox Controller
	 */
	public JoystickButton getRightBumper() {
		return new JoystickButton(this, 6);
	}
	
	public JoystickButton getSelect() {
		return new JoystickButton(this, 7);
	}
	
	public JoystickButton getStart() {
		return new JoystickButton(this, 8);
	}
	
	public JoystickButton getLStickButton() {
		return new JoystickButton(this, 9);
	}
	
	public JoystickButton getRStickButton() {
		return new JoystickButton(this, 10);
	}

}
