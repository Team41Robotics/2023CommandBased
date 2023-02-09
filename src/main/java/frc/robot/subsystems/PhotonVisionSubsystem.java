package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Transform2d;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
	static PhotonVisionSubsystem pv;

	ShuffleboardTab camtab = Shuffleboard.getTab("Camera");

	Transform2d[] taglocs = new Transform2d[] { // CHANGE WITH COORD SYSTEM
		null, new Transform2d(0, 4, 0), new Transform2d(0, 2.25, 0)
	};

	PhotonCamera[] cameras = new PhotonCamera[] {new PhotonCamera("TopCamera")};
	Transform2d[] camlocs = new Transform2d[] {new Transform2d(0, -0.12, 0)};
	double[] last_time = new double[] {Timer.getFPGATimestamp()};

	ArrayList<TimedPoseEstimate> ests = new ArrayList<>();

	public void update(int ci) {
		PhotonCamera cam = cameras[ci];
		PhotonPipelineResult res = cam.getLatestResult();
		if (res.getTimestampSeconds() > last_time[ci] && res.hasTargets()) {
			last_time[ci] = res.getTimestampSeconds();
			System.out.println("UPDate CI = " + ci);
			for (PhotonTrackedTarget tgt : res.getTargets()) {
				int id = tgt.getFiducialId();
				if (id == 0 || id > taglocs.length) continue;

				var bestt = tgt.getBestCameraToTarget().inverse();
				Transform2d best = new Transform2d(
						bestt.getX(), bestt.getY(), bestt.getRotation().getZ());
				Transform2d pose = taglocs[id].mul(best.mul(camlocs[ci]));

				var altt = tgt.getAlternateCameraToTarget().inverse();
				Transform2d alt = new Transform2d(
						altt.getX(), altt.getY(), altt.getRotation().getZ());
				Transform2d altpose = taglocs[id].mul(alt.mul(camlocs[ci]));

				System.out.println("FOUND TARGET id = " + id);
				TimedPoseEstimate est = new TimedPoseEstimate(pose, altpose, res.getTimestampSeconds());
				ests.add(est);
			}
		}
	}

	double last_eval_time = Timer.getFPGATimestamp();

	public void evaluate() {
		last_eval_time = Timer.getFPGATimestamp();
		System.out.println("\nESTS SUMMARY: ");
		for (int i = 0; i < ests.size(); i++) {
			System.out.println("EST: ");
			ests.get(i).pose.print();
			ests.get(i).altpose.print();
		}
		ests.clear();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < cameras.length; i++) update(i);
		// if (Timer.getFPGATimestamp() > last_eval_time + 0.2) evaluate();
	}

	public static class TimedPoseEstimate {
		public Transform2d pose;
		public Transform2d altpose;
		public double time;

		public TimedPoseEstimate(Transform2d pose, Transform2d altpose, double time) {
			this.pose = pose;
			this.altpose = altpose;
			this.time = time;
		}
	}

	public static PhotonVisionSubsystem getInstance() {
		if (pv == null) {
			pv = new PhotonVisionSubsystem();
		}
		return pv;
	}
}
