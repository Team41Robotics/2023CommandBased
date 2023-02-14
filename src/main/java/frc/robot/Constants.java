package frc.robot;

public final class Constants {
	public static final double FALCON_MAX_SPEED = 6380;
	public static final double NEO_MAX_SPEED = 5676;

	public static class OperatorConstants {

		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int LEFT_JOYSTICK_PORT = 1;
		public static final int RIGHT_JOYSTICK_PORT = 0;

		// TODO tuning; variable drive speed?
		public static final double FWD_DRIVE_VELOCITY = FALCON_MAX_SPEED / DrivetrainConstants.FWD_ROTS_PER_METER;
		public static final double H_DRIVE_VELOCITY = NEO_MAX_SPEED / DrivetrainConstants.H_ROTS_PER_METER;
		public static final double TURN_VELOCITY = FWD_DRIVE_VELOCITY / DrivetrainConstants.RADIUS;
	}

	public static class DrivetrainConstants {

		public static final int PORT_L1 = 0; // TODO TODO TODO TODO PORTS
		public static final int PORT_L2 = 0;
		public static final int PORT_M1 = 0;
		public static final int PORT_M2 = 0;
		public static final int PORT_R1 = 0;
		public static final int PORT_R2 = 0;

		public static final double FWD_RATIO = 9.75;
		public static final double H_RATIO = 10.65;

		public static final double FWD_WHEEL_RADIUS = 3 * 2.54 / 100;
		public static final double H_WHEEL_RADIUS = 2 * 2.54 / 100;

		public static final double FWD_WHEEL_TRACK = 2 * Math.PI * FWD_WHEEL_RADIUS;
		public static final double H_WHEEL_TRACK = 2 * Math.PI * H_WHEEL_RADIUS;

		public static final double FWD_ROTS_PER_METER = FWD_WHEEL_TRACK * FWD_RATIO;
		public static final double H_ROTS_PER_METER = H_WHEEL_TRACK * H_RATIO;

		public static final double RADIUS = 1; // TODO make this useful
	}

	public static final double GOTO_XY_THRESHOLD = 0.03;
	public static final double GOTO_TURN_THRESHOLD = 4 / 180. * Math.PI;
}
