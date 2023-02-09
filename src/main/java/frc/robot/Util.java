package frc.robot;

import frc.robot.Constants.OperatorConstants;

public class Util {
	/**
	 * @param joystickAxis joystick axis input
         * @param deadZone dead zone
	 * @return applied deadzone
	 */
	public static double deadZone(double joystickAxis, double deadZone) {
		return Math.abs(joystickAxis) > deadZone ? joystickAxis : 0;
	}

	/**
	 * @param joystickAxis joystick axis input
	 * @return applied deadzone with default deadzone
	 */
	public static double deadZone(double joystickAxis) {
		return deadZone(joystickAxis, OperatorConstants.JOYSTICK_DEADZONE);
	}

	/**
	 * @param rad
	 * @return rotation normalized from -180 deg to 180 deg
	 */
	public static double normRot(double rad) {
		rad %= 2 * Math.PI;
		rad += 2 * Math.PI;
		rad %= 2 * Math.PI;
		if (rad > Math.PI) rad -= 2 * Math.PI;
		return rad;
	}
}
