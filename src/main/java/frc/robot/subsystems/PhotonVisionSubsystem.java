package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Transform2d;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
	static PhotonVisionSubsystem pv;

	PhotonCamera cam = new PhotonCamera("TopCamera");
	ShuffleboardTab camtab = Shuffleboard.getTab("Camera");

	HashMap<Integer, Transform2d> taglocs = new HashMap<>(); // CHANGE WITH COORD SYSTEM
	Transform2d cam_to_com = new Transform2d(0, -0.12, 0);

	public PhotonVisionSubsystem() {
		// 16.5 x 8
		taglocs.put(1, new Transform2d(0, 4, 0)); // non field
		taglocs.put(2, new Transform2d(0, 3.25, 0));
		// field
		// taglocs.put(1, new Transform2d(15.513, 1.071, Math.PI));

		camtab.addNumber("px", () -> last_pose.getX());
		camtab.addNumber("py", () -> last_pose.getY());
		camtab.addNumber("ptheta", () -> last_pose.getTheta());
	}

	double last_time = Timer.getFPGATimestamp();
	Transform2d last_pose = new Transform2d();

	@Override
	public void periodic() {
		PhotonPipelineResult res = cam.getLatestResult();
		if (res.getTimestampSeconds() > last_time) {
			last_time = res.getTimestampSeconds();
			for (PhotonTrackedTarget tgt : res.getTargets()) {
				int id = tgt.getFiducialId();
				if (!taglocs.containsKey(id)) continue;
				var bestt = tgt.getBestCameraToTarget().inverse();
				var altt = tgt.getAlternateCameraToTarget().inverse();
				Transform2d best = new Transform2d(
						bestt.getX(), bestt.getY(), bestt.getRotation().getZ());
				Transform2d alt = new Transform2d(
						altt.getX(), altt.getY(), altt.getRotation().getZ());
				double ambig = tgt.getPoseAmbiguity();
				if (ambig < 0.2) {
					Transform2d pose = taglocs.get(id).mul(best.mul(cam_to_com));
					last_pose = pose;

					OdomSubsystem.getInstance().update_from(pose, last_time);
				}
			}
		}
	}

	public static PhotonVisionSubsystem getInstance() {
		if (pv == null) {
			pv = new PhotonVisionSubsystem();
		}
		return pv;
	}
}
