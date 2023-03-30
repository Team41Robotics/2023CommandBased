package frc.robot.constants;

import static java.lang.Math.PI;

import frc.robot.util.ArmPosition;

public final class Constants {
	public static class OperatorConstants {
		public enum HeldObject {
			CUBE,
			CONE,
			NONE
		}

		public static final int LEFT_JOYSTICK_PORT = 1;
		public static final int RIGHT_JOYSTICK_PORT = 0;

		public static final double JOYSTICK_DEADZONE = 0.2;
	}

	public static class GoToConstants {
		public static final double GOTO_XY_TOLERANCE = 0.05;
		public static final double GOTO_TURN_TOLERANCE = 3 / 180. * PI;
		public static final double GOTO_VEL_TOLERANCE = 0.2;
	}

	public enum LEDLocations {
		LEFT,
		RIGHT,
		MID,
		NONE
	}

	public enum ArmPos {
		BALL_TOP(1.000, 0.038, 1.203),
		BALL_MID(0.833, -0.574, 0.869),
		ALL_BOT(0.116, -0.850, 1.172),
		CONE_TOP(0.900, 0.104, 0.423),
		CONE_MID(0.40, 0.163, 0.294),
		CONE_PICKUP(0.150, -0.700, 0.740),
		BALL_PICKUP(0.000, -0.900, 1.000),
		BALL_PLATFORM(0.950, 0.249, 0.55),
		CONE_PLATFORM(0.950, 0.249, 0.0),
		CONE_SLIDE(0.050, -0.739, 1.808),
		BALL_SLIDE(0.140, -0.200, 1.523);

		public final double e;
		public final double j1;
		public final double j2;

		ArmPos(double e, double j1, double j2) {
			this.e = e;
			this.j1 = j1;
			this.j2 = j2;
		}

		public ArmPosition asPosition() {
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

		public static final int DIO_LOWERLIMIT = 0;
		public static final int DIO_JOINT2_LIMIT = 5;
	}

	public static final double LOOP_TIME = 0.02;

	public static final double FIELD_LENGTH = 16.54;
	public static final boolean[] isCone = new boolean[] {true, false, true, true, false, true, true, false, true};

	public static final double FIELD_WIDTH = 8;
}
