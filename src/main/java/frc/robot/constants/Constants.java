package frc.robot.constants;

import static java.lang.Math.PI;

import frc.robot.util.ArmPosition;

public final class Constants {
	public static class OperatorConstants {
		public static enum HeldObject {
			CUBE,
			CONE,
			NONE;
		}

		public static final int LEFT_JOYSTICK_PORT = 1;
		public static final int RIGHT_JOYSTICK_PORT = 0;

		public static final double JOYSTICK_DEADZONE = 0.2;

		// TODO tuning based on theoretical sysid'd values
		public static final double FWD_DRIVE_VELOCITY = 0;
		public static final double TURN_VELOCITY = 0;
	}

	public static class GoToConstants {
		public static final double GOTO_XY_TOLERANCE = 0.05;
		public static final double GOTO_TURN_TOLERANCE = 3 / 180. * PI;
		public static final double GOTO_VEL_TOLERANCE = 0.2;
	}

	public static enum LEDLocations {
		LEFT,
		RIGHT,
		MID,
		NONE;
	}

	public static enum ArmPos {
		BALL_TOP(1.000, 0.038, 1.203),
		BALL_MID(0.833, -0.574, 0.869),
		ALL_BOT(0.116, -0.850, 1.172),
		CONE_MID(0.500, 0.183, 0.374),
		CONE_TOP(1.000, 0.104, 0.753),
		CONE_PICKUP(0.141, 0.095, -0.661),
		BALL_PICKUP(0.000, -0.900, 0.900),
		BALL_PLATFORM(0.950, 0.249, 0.55),
		CONE_PLATFORM(0.950, 0.249, 0.0);
		public double e, j1, j2;

		private ArmPos(double e, double j1, double j2) {
			this.e = e;
			this.j1 = j1;
			this.j2 = j2;
		}

		public ArmPosition asPostion() {
			return new ArmPosition(this.e, this.j1, this.j2);
		}
	}

	public static class Ports {
		public static final int CAN_DT_L1 = 1;
		public static final int CAN_DT_L2 = 2;
		public static final int CAN_DT_M1 = 5;
		public static final int CAN_DT_M2 = 6;
		public static final int CAN_DT_R1 = 3;
		public static final int CAN_DT_R2 = 4;

		public static final int CAN_ELEV = 8;
		public static final int CAN_ELEV1 = 9;
		public static final int CAN_JOINT1 = 10;
		public static final int CAN_JOINT2 = 12;

		public static final int CAN_INTAKE = 13;

		public static final int DIO_LOWERLIMIT1 = 2;
		public static final int DIO_LOWERLIMIT2 = 3;
		public static final int DIO_UPPERLIMIT1 = 0;
		public static final int DIO_UPPERLIMIT2 = 1;
		public static final int DIO_JOINT2_LIMIT = 5;
	}

	public static final double LOOP_TIME = 0.02;

	public static final double FIELD_LENGTH = 16.54;
	public static final double FIELD_WIDTH = 8;
}
