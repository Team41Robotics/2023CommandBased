package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Transform2d;
import frc.robot.Util;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
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

	HashSet<Integer> ids = new HashSet<>();
	ArrayList<TimedPoseEstimate> bests = new ArrayList<>();
	ArrayList<TimedPoseEstimate> alts = new ArrayList<>();

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
				bests.add(new TimedPoseEstimate(pose, res.getTimestampSeconds()));
				alts.add(new TimedPoseEstimate(altpose, res.getTimestampSeconds()));
				ids.add(id * cameras.length + ci);
			}
		}
	}

	public EvaluationResult evaluateMask(int mask) {
		// TODO Auto-generated method stub
		double xtot = 0;
		double ytot = 0;
		double stot = 0;
		double ctot = 0;

		altmask.clear();

		for (int i = 0; i < bests.size(); i++) {
			if (i < 5) {
				// use altmask
				altmask.add(((mask >> i) & 1) == 1);
				TimedPoseEstimate est = (((mask >> i) & 1) == 1) ? alts.get(i) : bests.get(i);
				Transform2d trans = odom.origin_if(est.pose, est.time);

				xtot += trans.getX();
				ytot += trans.getY();
				stot += Math.sin(trans.getTheta());
				ctot += Math.cos(trans.getTheta());
			} else {
				double xavg = xtot / i;
				double yavg = ytot / i;
				double tavg = Math.atan2(stot, ctot);

				TimedPoseEstimate beste = bests.get(i);
				TimedPoseEstimate alte = alts.get(i);
				Transform2d best = odom.origin_if(beste.pose, beste.time);
				Transform2d alt = odom.origin_if(alte.pose, alte.time);

				double bs = Util.dist3(best.getX() - xavg, best.getY() - yavg, Util.normRot(best.getTheta() - tavg));
				double as = Util.dist3(alt.getX() - xavg, alt.getY() - yavg, Util.normRot(alt.getTheta() - tavg));
				if (as < bs) {
					best = alt;
					altmask.add(true);
				} else altmask.add(false);

				xtot += best.getX();
				ytot += best.getY();
				stot += Math.sin(best.getTheta());
				ctot += Math.cos(best.getTheta());
			}
		}
		EvaluationResult res = new EvaluationResult();
		double xavg = xtot / bests.size();
		double yavg = ytot / bests.size();
		double tavg = Math.atan2(stot, ctot);

		for (int i = 0; i < bests.size(); i++) {
			TimedPoseEstimate est = altmask.get(i) ? alts.get(i) : bests.get(i);
			Transform2d pose = odom.origin_if(est.pose, est.time);
			double dist = Util.dist3(pose.getX() - xavg, pose.getY() - yavg, Util.normRot(pose.getTheta() - tavg));
			res.stddev += dist;
		}
		res.stddev /= bests.size();
		res.mean = new Transform2d(xavg, yavg, tavg);
		return res;
	}

	public void evaluate() { // FIXME
		System.out.println("EVALUATING size of ids:" + ids.size() + " bests.size: " + bests.size());
		last_time = Timer.getFPGATimestamp();
		if (ids.size() > 1 && bests.size() > 5) {
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
		bests.clear();
		alts.clear();
		ids.clear();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < cameras.length; i++) update(i);
		if (Timer.getFPGATimestamp() > last_time + 0.2) evaluate();
	}

	public static class TimedPoseEstimate {
		public Transform2d pose;
		public double time;

		public TimedPoseEstimate(Transform2d pose, double time) {
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
