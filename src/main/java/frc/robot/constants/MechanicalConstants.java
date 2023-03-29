package frc.robot.constants;

import static java.lang.Math.PI;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.util.SystemIdentification;

public class MechanicalConstants {
	public static class DrivetrainConstants {
		public static final double FWD_RATIO = 9.75 / 12 * 14;
		public static final double H_RATIO = 10.65;

		public static final double FWD_WHEEL_RADIUS = 3 * 2.54 / 100;
		public static final double FWD_METERS_PER_ROT = FWD_WHEEL_RADIUS * 2 * PI;
		public static final double H_WHEEL_RADIUS = 2 * 2.54 / 100;

		public static final double FWD_METER_PER_RAD = FWD_WHEEL_RADIUS / FWD_RATIO;
		public static final double H_METER_PER_RAD = H_WHEEL_RADIUS / H_RATIO;

		public static final SystemIdentification FWD_IDENTF = new SystemIdentification(0, 0.20494, 2.6016, 0.13941);
		public static final SystemIdentification MID_IDENTF = new SystemIdentification(0, 0, 1, 0);

		public static final Constraints FWD_CONSTRAINTS = new Constraints(4, 100);
		public static final Constraints MID_CONSTRAINTS = new Constraints(12, 12);

		// public static final double RADIUS = 0.6512 / 2;
		public static final double RADIUS = 0.50833;
	}

	public static class ArmConstants {
		public static final double ELEV_RATIO = 5;
		public static final double JOINT1_RATIO = 81 * 84 / 16.;
		public static final double JOINT2_RATIO = 90 * 84 / 16.;

		public static final double ELEV_METERS_PER_AXLE_RAD = 0.0459 / 2 * 2;
		public static final double ELEV_RAD_PER_METER = ELEV_RATIO / ELEV_METERS_PER_AXLE_RAD;
		// this is only for sysid convenience
		public static final double METERS_PER_ROTATION = ELEV_METERS_PER_AXLE_RAD * 2 * PI;

		public static final double ELEV_THETA = 50 / 180. * PI;
		public static final double ELEV_LEN = 1;
		public static final double ARM_LEN = 14.75 * 2.54 / 100;

		public static final SystemIdentification ELEV_IDENTF =
				new SystemIdentification(0.40411, 0.01766, 2.4782, 0.10277);
		public static final SystemIdentification JOINT1_IDENTF =
				new SystemIdentification(0.69883, 0.27973, 4.042, 0.098954);
		public static final SystemIdentification JOINT2_IDENTF =
				new SystemIdentification(0.16105, 0.11592, 4.145, 0.0929);

		public static final Constraints ELEV_CONSTRAINTS = new Constraints(.2, 40);
		public static final Constraints JOINT1_CONSTRAINTS = new Constraints(.2, 40);
		public static final Constraints JOINT2_CONSTRAINTS = new Constraints(.2, 40);

		public static final double ELEVATOR_TOLERANCE = 0.03;
		public static final double JOINT_TOLERANCE = 2 / 180. * PI;

		public static final double JOINT1_TOPLIMIT_ANGLE = 16.15 / 180. * PI;
	}
}
