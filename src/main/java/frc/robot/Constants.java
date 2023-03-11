package frc.robot;

import static java.lang.Math.PI;

import edu.wpi.first.math.Pair;
import java.util.Map;

public final class Constants {
	public static final double FALCON_MAX_SPEED = 6380 * 2 * PI / 60;
	public static final double NEO_MAX_SPEED = 5676 * 2 * PI / 60;
	public static final double NEO_550_MAX_SPEED_RPM = 11710;

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
				DrivetrainConstants.LEFT_RAD_PER_METER / Constants.FALCON_MAX_SPEED; // TODO SYSID
		public static final double RIGHT_SPEED_TO_ONE =
				DrivetrainConstants.RIGHT_RAD_PER_METER / Constants.FALCON_MAX_SPEED;
		public static final double H_SPEED_TO_ONE = DrivetrainConstants.H_RAD_PER_METER / Constants.NEO_MAX_SPEED;

		public static final double RADIUS = 0.6512;
	}

	public static class ArmConstants {
		public static final int ELEV_ID = 9;
		public static final int ELEV1_ID = 8;
		public static final int JOINT1_ID = 10;
		public static final int JOINT2_ID = 13;

		public static final int LOWERLIMIT1_ID = 2;
		public static final int LOWERLIMIT2_ID = 3;
		public static final int UPPERLIMIT1_ID = 0;
		public static final int UPPERLIMIT2_ID = 1;

		public static final double ELEV_RATIO = 5;
		public static final double JOINT1_RATIO = 81 * 84 / 16.;
		public static final double JOINT2_RATIO = 81 * 84 / 16.;

		public static final double ELEV_METERS_PER_AXLE_RAD = 0.0459 / 2 * 2;
		public static final double TEMP = ELEV_METERS_PER_AXLE_RAD * 2 * PI;
		public static final double ELEV_RAD_PER_METER = ELEV_RATIO / ELEV_METERS_PER_AXLE_RAD;

		public static final double ELEV_THETA = 50 / 180. * PI;
		public static final double ELEV_LEN = 1;
		public static final double ARM_LEN = 14.75 * 2.54 / 100;

		public static final double ELEV_kV = 2.6369;
		public static final double ELEV_kS = 0.075079;
		public static final double ELEV_kA = 0.14281;
		public static final double ELEV_kG = 0.42523;

		public static final double JOINT1_kV = 4.7816;
		public static final double JOINT1_kS = 0.25931;
		public static final double JOINT1_kA = 0.44741;
		public static final double JOINT1_kG = 0.80517;

		public static final double JOINT2_kV = 4.145;
		public static final double JOINT2_kS = 0.11592;
		public static final double JOINT2_kA = 0.092903;
		public static final double JOINT2_kG = 0.16105;

		public static final double ELEV_MAX_SPEED = 0.4; // todo go figure
		public static final double ELEV_MAX_ACCEL = 35;
		public static final double JOINT1_MAX_SPEED = 18 / 180. * PI;
		public static final double JOINT1_MAX_ACCEL = 20;
		public static final double JOINT2_MAX_SPEED = 18 / 180. * PI;
		public static final double JOINT2_MAX_ACCEL = 55;

		public static final double ELEVATOR_TOLERANCE = 0.03;
		public static final double JOINT_TOLERANCE = 2 / 180. * PI;

		public static final double JOINT1_ZERO_THRES = 0.1;
		public static final double JOINT1_ZERO_ANGLE = 16.15 / 180. * PI;
		public static final double JOINT1_UPPER_BOUND = 15 / 180. * PI;
		public static final double JOINT1_LOWER_BOUND = -90 / 180. * PI;
	}

	public static class GoToConstants {
		public static final double GOTO_XY_THRESHOLD = 0.03;
		public static final double GOTO_TURN_THRESHOLD = .5 / 180. * PI;
		public static final double GOTO_VEL_THRES = 0.2;
	}

	public static class LEDConstants {

		public static final int LEFT_LENGTH = 44;
		public static final int MID_LENGTH = 20;
		public static final int RIGHT_LENGTH = 44;

		public enum LEDLocations {
			LEFT,
			MID,
			RIGHT
		}

		public static Map<LEDLocations, Pair<Integer, Integer>> lightPositions = Map.of(
				LEDLocations.LEFT, new Pair<Integer, Integer>(0, 45),
				LEDLocations.MID, new Pair<Integer, Integer>(45, 20),
				LEDLocations.RIGHT, new Pair<Integer, Integer>(66, 42));
	}

	public static final double LOOP_TIME = 0.02;
	public static final int INTAKE_ID = 12;

	public static final double FIELD_LENGTH = 16.54;
	public static final double FIELD_WIDTH = 8;
}
