package frc.robot.constants;

import static java.lang.Math.PI;

// TODO values with 0 are probably wrong

public final class Constants {
	public static class OperatorConstants {
		public static enum HeldObject {
			CUBE("Cube"),
			CONE("Cone"),
			NONE("None");
			public String name;

			private HeldObject(String name) {
				this.name = name;
			}
		}

		public static final int LEFT_JOYSTICK_PORT = 1;
		public static final int RIGHT_JOYSTICK_PORT = 0;

		public static final double JOYSTICK_DEADZONE = 0.2;

		// TODO tuning; variable drive speed? after sysid; just arbitrary
		public static final double FWD_DRIVE_VELOCITY = 0;
		public static final double TURN_VELOCITY = 0;
	}

	public static class GoToConstants {
		public static final double GOTO_XY_TOLERANCE = 0.07;
		public static final double GOTO_TURN_TOLERANCE = .5 / 180. * PI;
		public static final double GOTO_VEL_TOLERANCE = 0.2;
	}

	public static enum LEDLocations {
		LEFT,
		RIGHT,
		MID,
		NONE;
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
		public static final int CAN_JOINT11 = 0; // TODO
		public static final int CAN_JOINT2 = 13;

		public static final int CAN_INTAKE = 12;

		public static final int DIO_LOWERLIMIT1 = 2;
		public static final int DIO_LOWERLIMIT2 = 3;
		public static final int DIO_UPPERLIMIT1 = 0;
		public static final int DIO_UPPERLIMIT2 = 1;
	}

	public static final double LOOP_TIME = 0.02;

	public static final double FIELD_LENGTH = 16.54;
	public static final double FIELD_WIDTH = 8;

	public static final double VISION_THETA_THRESHOLD = 10 / 180. * PI;
}
