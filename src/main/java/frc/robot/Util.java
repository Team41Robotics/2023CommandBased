package frc.robot;

public class Util {
	public static double deadZone(double joystickAxis) {
		return Math.abs(joystickAxis) > 0.2 ? joystickAxis : 0;
	}

	public static double normRot(double rad) {
		rad %= 2 * Math.PI;
		rad += 2 * Math.PI;
		rad %= 2 * Math.PI;
		if (rad > Math.PI) rad -= 2 * Math.PI;
		return rad;
	}

	public static Transform2d lerp(Transform2d t1, Transform2d t2, double alpha) {
		double tx = (1 - alpha) * t1.x + alpha * t2.x;
		double ty = (1 - alpha) * t1.y + alpha * t2.y;
		double theta = t1.theta + alpha * normRot(t2.theta - t1.theta);

		return new Transform2d(tx, ty, theta);
	}
}
