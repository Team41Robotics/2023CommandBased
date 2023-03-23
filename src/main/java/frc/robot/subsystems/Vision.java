package frc.robot.subsystems;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDSegment;
import frc.robot.util.Transform2d;
import java.io.IOException;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
	Transform2d[] taglocs = new Transform2d[] { // CHANGE WITH COORD SYSTEM
		null,
		new Transform2d(15.51310, 1.06341, PI),
		new Transform2d(15.51310, 2.73981, PI),
		new Transform2d(15.51310, 4.41621, PI),
		new Transform2d(16.17832, 6.74161, PI),
		new Transform2d(0.36168, 6.74161, 0),
		new Transform2d(1.02690, 4.41621, 0),
		new Transform2d(1.02690, 2.73981, 0),
		new Transform2d(1.02690, 1.06341, 0),
	};

	PhotonCamera[] cameras = new PhotonCamera[] {new PhotonCamera("HD_USB_Camera")};
	Transform2d[] camlocs = new Transform2d[] {new Transform2d(14 * 2.54 / 100, -11.5 * 2.54 / 100, 0)};
	double[] last_time = new double[] {Timer.getFPGATimestamp()};

	Transform2d[] poses = new Transform2d[32];
	double[] times = new double[32];
	double[] areas = new double[32];
	int ptr = 0;

	public void init() {}

	public void update(int ci) {
		PhotonCamera cam = cameras[ci];
		PhotonPipelineResult res = cam.getLatestResult();
		double time = res.getTimestampSeconds();

		if (time > last_time[ci] && res.hasTargets()) {
			last_time[ci] = time;
			try {
				PhotonPoseEstimator poseestimator = new PhotonPoseEstimator(
						AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
						PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
						cam,
						new Transform3d(
								new Translation3d(camlocs[ci].x, camlocs[ci].y, 0),
								new Rotation3d(0, 0, camlocs[ci].theta)));

				poseestimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
				if (odom.origin.x == 0 && odom.origin.y == 0 && odom.origin.theta == 0)
					poseestimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

				poseestimator.setReferencePose(
						new Pose2d(odom.now().x, odom.now().y, new Rotation2d(odom.now().theta)));
				EstimatedRobotPose pose = poseestimator.update(res).orElse(null);
				if (pose != null) {
					poses[ptr % 32] = new Transform2d(
							pose.estimatedPose.getX(),
							pose.estimatedPose.getY(),
							pose.estimatedPose.getRotation().getZ());
					times[ptr % 32] = time;
					areas[ptr % 32] = pose.targetsUsed.stream()
							.mapToDouble(x -> x.getArea())
							.sum();
					ptr++;
				}
			} catch (IOException e) {
				// Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	public void evaluate() { // assume all tags are correct and then unweighted avg
		double tx = 0;
		double ty = 0;
		double tsin = 0; // orz
		double tcos = 0;

		int sz = 0;
		double totarea = 0;
		if (ptr == 0) return;
		for (int i = 0; i < 32 && i < ptr; i++) {
			if (Timer.getFPGATimestamp() - times[i] > 0.5) continue;
			sz++;
			Transform2d o = odom.origin_if(poses[i], times[i]);
			tx += o.x * areas[i];
			ty += o.y * areas[i];
			tsin += o.sin * areas[i];
			tcos += o.cos * areas[i];
			totarea += areas[i];
		}
		ptr = 0;
		double norm = sqrt(tsin * tsin + tcos * tcos);
		Transform2d avg = new Transform2d(tx / totarea, ty / totarea, tcos / norm, tsin / norm);

		if (sz > 3 && DriverStation.isDisabled()) LEDSegment.midSide.setColor(Color.kGreen);
		else if (DriverStation.isDisabled()) LEDSegment.midSide.flashColor(Color.kRed);
		if (sz > 3) odom.update_origin(avg);
	}

	@Override
	public void periodic() {
		for (int i = 0; i < cameras.length; i++) update(i);
		evaluate();
	}
}
