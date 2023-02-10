package frc.robot;

public final class Constants {
	public static class OperatorConstants {

		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int LEFT_JOYSTICK_PORT = 1;
		public static final int RIGHT_JOYSTICK_PORT = 0;

		public static final double FWD_DRIVE_VELOCITY = 6380 / DrivetrainConstants.FWD_ROTS_PER_METER; // TODO
		public static final double H_DRIVE_VELOCITY = 5676 / DrivetrainConstants.H_ROTS_PER_METER;
                public static final double TURN_VELOCITY = FWD_DRIVE_VELOCITY / DrivetrainConstants.RADIUS; // TODO FIXME TODO FIXME
	}

	public static class DrivetrainConstants {

		public static final int BOTTOM_LEFT = 0;
		public static final int TOP_LEFT = 1;
		public static final int MID = 2;
		public static final int TOP_RIGHT = 3;
		public static final int BOTTOM_RIGHT = 4;

		public static final int BACK_LEFT_MOTOR_CHANNEL = 0;
		public static final int BACK_RIGHT_MOTOR_CHANNEL = 4;
		public static final int FRONT_LEFT_MOTOR_CHANNEL = 1;
		public static final int FRONT_RIGHT_MOTOR_CHANNEL = 3;
		public static final int MID_MOTOR_CHANNEL = 2;

		public static final double FWD_RATIO = 9.75;
		public static final double H_RATIO = 10.65;

		public static final double FWD_WHEEL_RADIUS = 3 * 2.54 / 100;
		public static final double H_WHEEL_RADIUS = 2 * 2.54 / 100;

		public static final double FWD_WHEEL_TRACK = 2 * Math.PI * FWD_WHEEL_RADIUS;
		public static final double H_WHEEL_TRACK = 2 * Math.PI * H_WHEEL_RADIUS;

		public static final double FWD_ROTS_PER_METER = FWD_WHEEL_TRACK * FWD_RATIO;
		public static final double H_ROTS_PER_METER = H_WHEEL_TRACK * H_RATIO;

                public static final double RADIUS = 1; // TODO
	}
}
