package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Transform2d;
import frc.robot.Util;
import java.util.ArrayList;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
	static PhotonVisionSubsystem pv;

	OdomSubsystem odom = OdomSubsystem.getInstance();

	// PhotonCamera topcam = new PhotonCamera("TopCamera");
	// ShuffleboardTab camtab = Shuffleboard.getTab("Camera");
	PhotonCamera[] cameras = new PhotonCamera[] {new PhotonCamera("TopCamera")};

	HashMap<Integer, Transform2d> taglocs = new HashMap<>(); // CHANGE WITH COORD SYSTEM
	HashMap<Integer, Transform2d> camlocs = new HashMap<>();

	public PhotonVisionSubsystem() {
		// 16.5 x 8
		taglocs.put(1, new Transform2d(0, 4, 0)); // non field
		taglocs.put(2, new Transform2d(0, 3.25, 0));
		// field
		// taglocs.put(1, new Transform2d(15.513, 1.071, Math.PI));

		camlocs.put(0, new Transform2d(0, -0.12, 0));
	}

	HashMap<Integer, TimedPoseEstimate> ids = new HashMap<>();
	ArrayList<TimedPoseEstimate> ests = new ArrayList<>();

	ArrayList<Boolean> altmask = new ArrayList<>();

	double last_time = Timer.getFPGATimestamp();

	public void update(int ci) {
		PhotonCamera cam = cameras[ci];
		PhotonPipelineResult res = cam.getLatestResult();
		if (res.hasTargets()) {
			for (PhotonTrackedTarget tgt : res.getTargets()) {
				int id = tgt.getFiducialId();
				if (!taglocs.containsKey(id)) continue;

				var bestt = tgt.getBestCameraToTarget().inverse();
				Transform2d best = new Transform2d(
						bestt.getX(), bestt.getY(), bestt.getRotation().getZ());
				Transform2d pose = taglocs.get(id).mul(best.mul(camlocs.get(ci)));

				var altt = tgt.getAlternateCameraToTarget().inverse();
				Transform2d alt = new Transform2d(
						altt.getX(), altt.getY(), altt.getRotation().getZ());
				Transform2d altpose = taglocs.get(id).mul(alt.mul(camlocs.get(ci)));

				TimedPoseEstimate est = new TimedPoseEstimate(pose, altpose, res.getTimestampSeconds());
				ests.add(est);
				ids.put(id * cameras.length + ci, est);
			}
		}
	}

	public EvaluationResult evaluateMask(int mask) {
		double xtot = 0;
		double ytot = 0;
		double stot = 0;
		double ctot = 0;

		altmask.clear();

		for (int i = 0; i < ests.size(); i++) {
			if (i < 5) {
				// use altmask
				altmask.add(((mask >> i) & 1) == 1);
				TimedPoseEstimate est = ests.get(i);
				Transform2d trans = odom.origin_if(((mask >> i) & 1) == 1 ? est.altpose : est.pose, est.time);

				xtot += trans.x;
				ytot += trans.y;
				stot += Math.sin(trans.theta);
				ctot += Math.cos(trans.theta);
			} else {
				double xavg = xtot / i;
				double yavg = ytot / i;
				double tavg = Math.atan2(stot, ctot);

				TimedPoseEstimate est = ests.get(i);
				Transform2d best = odom.origin_if(est.pose, est.time);
				Transform2d alt = odom.origin_if(est.altpose, est.time);

				double bs = Util.dist3(best.x - xavg, best.y - yavg, Util.normRot(best.theta - tavg));
				double as = Util.dist3(alt.x - xavg, alt.y - yavg, Util.normRot(alt.theta - tavg));
				if (as < bs) {
					best = alt;
					altmask.add(true);
				} else altmask.add(false);

				xtot += best.x;
				ytot += best.y;
				stot += Math.sin(best.theta);
				ctot += Math.cos(best.theta);
			}
		}
		EvaluationResult res = new EvaluationResult();
		double xavg = xtot / ests.size();
		double yavg = ytot / ests.size();
		double tavg = Math.atan2(stot, ctot);

		for (int i = 0; i < ests.size(); i++) {
			TimedPoseEstimate est = ests.get(i);
			Transform2d pose = odom.origin_if(altmask.get(i) ? est.altpose : est.pose, est.time);
			double dist = Util.dist3(pose.x - xavg, pose.y - yavg, Util.normRot(pose.theta - tavg));
			res.stddev += dist;
		}
		res.stddev /= ests.size();
		res.mean = new Transform2d(xavg, yavg, tavg);
		return res;
	}

	public void evaluate() { // FIXME
		System.out.println("EVALUATING size of ids:" + ids.size() + " ests.size: " + ests.size());
		last_time = Timer.getFPGATimestamp();
		if (ids.size() > 1 && ests.size() > 5) {
			EvaluationResult bestscore = evaluateMask(0);
			for (int mask = 1; mask < 32; mask++) {
				EvaluationResult score = evaluateMask(mask);
				if (score.stddev < bestscore.stddev) {
					bestscore = score;
				}
			}
			// TODO: update odom origin
			System.out.println("ODOM ORIGIN UPDATED stddev: " + bestscore.stddev);
			odom.update_origin(bestscore.mean);
		}
		// done
		ests.clear();
		ids.clear();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < cameras.length; i++) update(i);
		if (Timer.getFPGATimestamp() > last_time + 0.2) evaluate();
	}

	public static class TimedPoseEstimate {
		public Transform2d pose;
		public Transform2d altpose;
		public double time;

		public TimedPoseEstimate(Transform2d pose, Transform2d altpose, double time) {
			this.pose = pose;
			this.time = time;
		}
	}

	public static class EvaluationResult {
		public Transform2d mean;
		public double stddev;
	}

	public static PhotonVisionSubsystem getInstance() {
		if (pv == null) {
			pv = new PhotonVisionSubsystem();
		}
		return pv;
	}
}
