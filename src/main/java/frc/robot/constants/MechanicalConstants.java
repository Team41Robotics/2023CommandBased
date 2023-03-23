package frc.robot.constants;

import static java.lang.Math.PI;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.util.SystemIdentification;

public class MechanicalConstants {
	public static class DrivetrainConstants {
		// TODO: can we replace pinions? then we can consolidate
		public static final double LEFT_RATIO = 9.75;
		public static final double RIGHT_RATIO = LEFT_RATIO * 14.0 / 12.0;
		public static final double H_RATIO = 10.65;

		public static final double FWD_WHEEL_RADIUS = 3 * 2.54 / 100;
		public static final double H_WHEEL_RADIUS = 2 * 2.54 / 100;

		public static final double LEFT_METER_PER_RAD = FWD_WHEEL_RADIUS / LEFT_RATIO;
		public static final double RIGHT_METER_PER_RAD = FWD_WHEEL_RADIUS / RIGHT_RATIO;
		public static final double H_METER_PER_RAD = H_WHEEL_RADIUS / H_RATIO;

		public static final SystemIdentification LEFT_IDENTF = new SystemIdentification(0, 0, 0, 0);
		public static final SystemIdentification RIGHT_IDENTF = new SystemIdentification(0, 0, 0, 0);
		public static final SystemIdentification MID_IDENTF = new SystemIdentification(0, 0, 0, 0);

		public static final Constraints LEFT_CONSTRAINTS = new Constraints(0, 0);
		public static final Constraints RIGHT_CONSTRAINTS = new Constraints(0, 0);
		public static final Constraints MID_CONSTRAINTS = new Constraints(0, 0);

		public static final double RADIUS = 0.6512 / 2;
	}

	public static class ArmConstants {
		public static final double ELEV_RATIO = 5;
		public static final double JOINT1_RATIO = 81 * 84 / 16.; // TODO change
		public static final double JOINT2_RATIO = 81 * 84 / 16.;

		public static final double ELEV_METERS_PER_AXLE_RAD = 0.0459 / 2 * 2;
		public static final double ELEV_RAD_PER_METER = ELEV_RATIO / ELEV_METERS_PER_AXLE_RAD;
		// TODO DELETE this is only for sysid convenience
		public static final double METERS_PER_ROTATION = ELEV_METERS_PER_AXLE_RAD * 2 * PI;

		public static final double ELEV_THETA = 50 / 180. * PI;
		public static final double ELEV_LEN = 1;
		public static final double ARM_LEN = 14.75 * 2.54 / 100;

		public static final SystemIdentification ELEV_IDENTF = new SystemIdentification(0, 0, 0, 0);
		public static final SystemIdentification JOINT1_IDENTF = new SystemIdentification(0, 0, 0, 0);
		public static final SystemIdentification JOINT2_IDENTF = new SystemIdentification(0, 0, 0, 0);

		public static final Constraints ELEV_CONSTRAINTS = new Constraints(0, 0);
		public static final Constraints JOINT1_CONSTRAINTS = new Constraints(0, 0);
		public static final Constraints JOINT2_CONSTRAINTS = new Constraints(0, 0);

		public static final double ELEVATOR_TOLERANCE = 0.03;
		public static final double JOINT_TOLERANCE = 2 / 180. * PI;

		public static final double JOINT1_TOPLIMIT_ANGLE = 16.15 / 180. * PI;
	}
}
