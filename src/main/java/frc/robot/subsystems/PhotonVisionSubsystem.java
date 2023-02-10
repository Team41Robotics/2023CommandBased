package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Transform2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
	private static final double THETA_THRESHOLD = 10 / 180. * Math.PI;

	static PhotonVisionSubsystem pv;

	OdomSubsystem odom = OdomSubsystem.getInstance();
	ShuffleboardTab camtab = Shuffleboard.getTab("Camera");

	Transform2d[] taglocs = new Transform2d[] { // CHANGE WITH COORD SYSTEM
		null, new Transform2d(0, 4, 0), new Transform2d(0, 2.25, 0)
	};

	PhotonCamera[] cameras = new PhotonCamera[] {new PhotonCamera("TopCamera")};
	Transform2d[] camlocs = new Transform2d[] {new Transform2d(0, -0.12, 0)};
	double[] last_time = new double[] {Timer.getFPGATimestamp()};

	public void update(int ci) {
		PhotonCamera cam = cameras[ci];
		PhotonPipelineResult res = cam.getLatestResult();
		double time = res.getTimestampSeconds();
		if (time > last_time[ci] && res.hasTargets()) {
			last_time[ci] = time;
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

				if (Math.abs(pose.theta - odom.now().theta) < THETA_THRESHOLD) {
					odom.update_from(pose, time);
				} else {
					if (Math.abs(altpose.theta - odom.now().theta) < THETA_THRESHOLD) {
						odom.update_from(pose, time);
					}
				}
			}
		}
	}

	double last_eval_time = Timer.getFPGATimestamp();

	@Override
	public void periodic() {
		for (int i = 0; i < cameras.length; i++) update(i);
	}

	public static PhotonVisionSubsystem getInstance() {
		if (pv == null) {
			pv = new PhotonVisionSubsystem();
		}
		return pv;
	}
}
