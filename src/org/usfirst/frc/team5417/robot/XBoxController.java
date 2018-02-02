package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.Joystick;

public class XBoxController {

	Joystick driverStick;
	boolean wasLBPressed = false;
	boolean LBbuttonPress = false;
	boolean wasRBPressed = false;
	boolean RBbuttonPress = false;
	boolean wasBackPressed = false;
	boolean backButtonPressed = false;
	boolean YButtonPressed = false;
	boolean wasYPressed = false;

	public XBoxController(Joystick driverStick) {
		this.driverStick = driverStick;
	}

	public boolean isLBHeldDown() {
		return driverStick.getRawButton(5);
	}

	public boolean isFirstLBPressed() {
		if (driverStick.getRawButton(5)) {
			if (wasLBPressed)
				LBbuttonPress = false;
			else
				LBbuttonPress = true;

			wasLBPressed = true;
		}
		
		else {
			wasLBPressed = false;
			LBbuttonPress = false;
		}
		return LBbuttonPress;
		
	}
	
	public boolean isFirstRBPressed() {
		if (driverStick.getRawButton(6)) {
			if (wasRBPressed)
				RBbuttonPress = false;
			else
				RBbuttonPress = true;

			wasRBPressed = true;
		}
		
		else {
			wasRBPressed = false;
			RBbuttonPress = false;
		}
		return RBbuttonPress;
	}
	public boolean isAHeldDown() {
		return driverStick.getRawButton(1);
		
	}
	public boolean isBHeldDown() {
		return driverStick.getRawButton(2);
	}
	public boolean isXHeldDown() {
		return driverStick.getRawButton(3);
		
	}
	public boolean isYHeldDown() {
		return driverStick.getRawButton(4);
		
	}
	public boolean isRBHeldDown() {
		return driverStick.getRawButton(6);
		
	}
	public double getLXValue() {
		return driverStick.getRawAxis(0);
	}
	public double getLYValue() {
		return driverStick.getRawAxis(1);
	}
	public double getLTValue() {
		return driverStick.getRawAxis(2);
	}
	public double getRTValue() {
		return driverStick.getRawAxis(3);
	}
	public double getRXValue() {
		return driverStick.getRawAxis(4);
	}
	public double getRYValue() {
		return driverStick.getRawAxis(5);
	}
	public boolean isStartHeldDown() {
		return driverStick.getRawButton(8);
	}
	public boolean isBackHeldDown() {
		return driverStick.getRawButton(7);
	}
	public boolean isDPadUpHeldDown() {
		return driverStick.getPOV(0) == 0;
	}
	public boolean isFirstBackPressed() {
		if (driverStick.getRawButton(7)) {
			if (wasBackPressed)
				backButtonPressed = false;
			else
				backButtonPressed = true;

			wasBackPressed = true;
		}
		
		else {
			wasBackPressed = false;
			backButtonPressed = false;
		}
		return backButtonPressed;
		
	}
	public boolean isFirstYPressed() {
		if (driverStick.getRawButton(4)) {
			if (wasYPressed)
				YButtonPressed = false;
			else
				YButtonPressed = true;

			wasYPressed = true;
		}
		
		else {
			wasYPressed = false;
			YButtonPressed = false;
		}
		return YButtonPressed;
		
	}
}