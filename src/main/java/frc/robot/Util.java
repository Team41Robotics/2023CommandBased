package frc.robot;

import frc.robot.Constants.OperatorConstants;

public class Util {
	/**
	 * @param joystickAxis joystick axis input
	 * @param deadZone dead zone
	 * @return applied deadzone
	 */
	public static double deadZone(double joystickAxis, double deadZone) {
		if (Math.abs(joystickAxis) < deadZone) return 0;
		return Math.signum(joystickAxis) * (Math.abs(joystickAxis) - deadZone) / (1 - deadZone);
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

	/**
	 * @param t1 1st transform to be lerped
	 * @param t2 2nd transform to be lerped
	 * @param alpha alpha value for lerping (0=t1; 1=t2)
	 * @return
	 */
	public static Transform2d lerp(Transform2d t1, Transform2d t2, double alpha) {
		double tx = (1 - alpha) * t1.x + alpha * t2.x;
		double ty = (1 - alpha) * t1.y + alpha * t2.y;
		double theta = t1.theta + alpha * normRot(t2.theta - t1.theta);

		return new Transform2d(tx, ty, theta);
	}
}
