package frc.robot;

public final class Constants {
	public static final double FALCON_MAX_SPEED = 6380 * 2 * Math.PI / 60;
	public static final double NEO_MAX_SPEED = 5676 * 2 * Math.PI / 60;

	public static class OperatorConstants {

		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int LEFT_JOYSTICK_PORT = 1;
		public static final int RIGHT_JOYSTICK_PORT = 0;

		public static final double JOYSTICK_DEADZONE = 0.2;

		// TODO tuning; variable drive speed? after sysid
		public static final double FWD_DRIVE_VELOCITY = FALCON_MAX_SPEED / DrivetrainConstants.LEFT_RAD_PER_METER;
		public static final double TURN_VELOCITY = FWD_DRIVE_VELOCITY / DrivetrainConstants.RADIUS;
	}

	public static class DrivetrainConstants {

		public static final int PORT_L1 = 1;
		public static final int PORT_L2 = 2;
		public static final int PORT_M1 = 5;
		public static final int PORT_M2 = 6;
		public static final int PORT_R1 = 3;
		public static final int PORT_R2 = 4;

		private static final double MECHANICAL_DRIFT_COMP = 0.95; // TODO: figure out cause of this // sysid may fix
		public static final double LEFT_RATIO = 9.75;
		public static final double RIGHT_RATIO = LEFT_RATIO * 14.0 / 12.0 * MECHANICAL_DRIFT_COMP;
		public static final double H_RATIO = 10.65;

		public static final double FWD_WHEEL_RADIUS = 3 * 2.54 / 100;
		public static final double H_WHEEL_RADIUS = 2 * 2.54 / 100;

		public static final double LEFT_RAD_PER_METER = 1 / FWD_WHEEL_RADIUS * LEFT_RATIO;
		public static final double RIGHT_RAD_PER_METER = 1 / FWD_WHEEL_RADIUS * RIGHT_RATIO;
		public static final double H_RAD_PER_METER = 1 / H_WHEEL_RADIUS * H_RATIO;

		public static final double LEFT_SPEED_TO_ONE =
				DrivetrainConstants.LEFT_RAD_PER_METER / Constants.FALCON_MAX_SPEED;
		public static final double RIGHT_SPEED_TO_ONE =
				DrivetrainConstants.RIGHT_RAD_PER_METER / Constants.FALCON_MAX_SPEED;
		public static final double H_SPEED_TO_ONE = DrivetrainConstants.H_RAD_PER_METER / Constants.NEO_MAX_SPEED;

		public static final double RADIUS = 0.6512;
	}

	public static final double GOTO_XY_THRESHOLD = 0.03;
	public static final double GOTO_TURN_THRESHOLD = .5 / 180. * Math.PI;
	public static final double GOTO_VEL_THRES = 0.2;
}
