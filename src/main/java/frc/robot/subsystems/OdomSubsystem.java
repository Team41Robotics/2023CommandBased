package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Transform2d;
import frc.robot.Util;
import java.util.ArrayList;

public class OdomSubsystem extends SubsystemBase {
	ArrayList<Double> times = new ArrayList<>();
	ArrayList<Transform2d> odoms = new ArrayList<>();
	Transform2d origin = new Transform2d(1.02690 + 0.84, 2.73981, Math.PI); // CHANGE WITH COORD SYSTEM
	// TODO ORIGIN; we can likely just use apriltags lol

	static OdomSubsystem odom;

	ShuffleboardTab odomstab = Shuffleboard.getTab("Odom");
	HDriveSubsystem hdrive = HDriveSubsystem.getInstance();

	public OdomSubsystem() {
		times.ensureCapacity(7000);
		odoms.ensureCapacity(7000);
		odomstab.addNumber("x", () -> now().x);
		odomstab.addNumber("y", () -> now().y);
		odomstab.addNumber("theta", () -> now().theta);
		odomstab.addNumber("Ox", () -> origin.x);
		odomstab.addNumber("Oy", () -> origin.y);
		odomstab.addNumber("Otheta", () -> origin.theta);
		odomstab.addNumber("Ax", () -> acc().x);
		odomstab.addNumber("Ay", () -> acc().y);
		odomstab.addNumber("Atheta", () -> acc().theta);
		odoms.add(new Transform2d(0, 0, 0));
		times.add(Timer.getFPGATimestamp());
	}

	public boolean isStarted = false;

	public void start() {
		isStarted = true;
		ptheta = Robot.imu.getAngle();
	}

	double ptheta = Robot.imu.getAngle();
	double pl_enc = hdrive.getLeftPos();
	double pm_enc = hdrive.getMidPos();
	double pr_enc = hdrive.getRightPos();

	@Override
	public void periodic() {
		double theta = Robot.imu.getAngle();
		double dtheta = theta - ptheta;
		ptheta = theta;

		double lenc = hdrive.getLeftPos();
		double menc = hdrive.getMidPos();
		double renc = hdrive.getRightPos();
		double dl = lenc - pl_enc;
		double ds = menc - pm_enc;
		double dr = renc - pr_enc;
		pl_enc = lenc;
		pm_enc = menc;
		pr_enc = renc;
		double df = (dl + dr) / 2;
		if (!isStarted) return;

		double dx, dy;
		if (dtheta < 1e-9) {
			dx = df;
			dy = ds;
		} else {
			dx = Math.sin(dtheta) / dtheta * df + (Math.cos(dtheta) - 1) / dtheta * ds;
			dy = (1 - Math.cos(dtheta)) / dtheta * df + Math.sin(dtheta) / dtheta * ds;
		}

		Transform2d trans = new Transform2d(dx, dy, dtheta);
		odoms.add(acc().mul(trans));
		times.add(Timer.getFPGATimestamp());
	}

	public Transform2d acc() {
		return odoms.get(odoms.size() - 1);
	}

	public Transform2d now() {
		return origin.mul(acc());
	}

	public Transform2d raw_get(double time) {
		// binary search on nearest odoms measurement
		int l = 0;
		int r = times.size() - 1;
		while (l < r) {
			int mid = l + (r - l) / 2;
			if (times.get(mid) < time) l = mid + 1;
			else r = mid;
		}
		if (r == 0) return new Transform2d();
		double tl = times.get(r - 1);
		double tr = times.get(r);
		return Util.lerp(odoms.get(r - 1), odoms.get(r), (time - tl) / (tr - tl));
	}

	public Transform2d get(double time) {
		return origin.mul(raw_get(time));
	}

	public Transform2d delta(double time) {
		return raw_get(time).inv().mul(acc());
	}

	public void update_origin(Transform2d origin) {
		this.origin = origin;
	}

	public Transform2d origin_if(Transform2d pose, double time) {
		return pose.mul(raw_get(time).inv());
	}

	public static OdomSubsystem getInstance() {
		if (odom == null) {
			odom = new OdomSubsystem();
		}
		return odom;
	}
}
