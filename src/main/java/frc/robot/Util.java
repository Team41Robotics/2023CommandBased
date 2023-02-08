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

	public static double dist(double dx, double dy) {
		return Math.sqrt(dx * dx + dy * dy);
	}

	public static double dist3(double dx, double dy, double dz) {
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	public static int randint(int l, int r) {
		return (int) (Math.floor(Math.random() * (r - l)) + l);
	}
}
