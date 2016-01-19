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
	 * @return a double value representing the horizontal position of the Left Stick on the XBox Controller.
	 */
	public double getLeftStickXAxis() {
		// return getX();
		return getRawAxis(0);
	}
	
	/**
	 * 
	 * @return a double value representing the vertical position of the Left Stick on the XBox Controller.
	 */
	public double getLeftStickYAxis() {
		// return getY();
		return getRawAxis(1);
	}
	
	/**
	 * 
	 * @return a double value representing the position of the Left Trigger on the XBox Controller.
	 */
	public double getLeftTriggerAxis() {
		return getRawAxis(2);
	}
	
	/**
	 * 
	 * @return a double value representing the position of the Right Trigger on the XBox Controller.
	 */
	public double getRightTriggerAxis() {
		return getRawAxis(3);
	}
	
	/**
	 * 
	 * @return a double value representing the horizontal position of the Right Stick on the XBox Controller.
	 */
	public double getRightStickXAxis() {
		return getRawAxis(4);
	}
	
	/**
	 * 
	 * @return a double value representing the vertical position of the Right Stick on the XBox Controller.
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
	
	/**
	 * This method returns the select button from the XBox Controller
	 * @return JoystickButton Mapped to button select on Xbox Controller
	 */
	public JoystickButton getSelect() {
		return new JoystickButton(this, 7);
	}
	
	/**
	 * This method returns the start button from the XBox Controller
	 * @return JoystickButton Mapped to button start on Xbox Controller
	 */
	public JoystickButton getStart() {
		return new JoystickButton(this, 8);
	}
	
	/**
	 * This method returns the left joystick button from the XBox Controller
	 * @return JoystickButton Mapped to button left joystick on Xbox Controller
	 */
	public JoystickButton getLStickButton() {
		return new JoystickButton(this, 9);
	}
	
	/**
	 * This method returns the right joystick button from the XBox Controller
	 * @return JoystickButton Mapped to button right joystick on Xbox Controller
	 */
	public JoystickButton getRStickButton() {
		return new JoystickButton(this, 10);
	}

}
