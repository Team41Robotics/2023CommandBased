package frc.robot.subsystems;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs.LEDSegment;
import frc.robot.util.Transform2d;
import frc.robot.util.Util;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
	Transform2d[] camlocs = new Transform2d[] {new Transform2d(-14 * 2.54 / 100, 11.5 * 2.54 / 100, 0)};
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
		if (!res.hasTargets() && DriverStation.isDisabled()) {
			LEDSegment.midSide.flashColor(Color.kRed);
		}
		if (res.hasTargets() && DriverStation.isDisabled()) {
			LEDSegment.midSide.setColor(Color.kGreen);
		}
		if (time > last_time[ci] && res.hasTargets()) {
			last_time[ci] = time;
			for (PhotonTrackedTarget tgt : res.getTargets()) {
				int id = tgt.getFiducialId();
				if (id == 0 || id >= taglocs.length) continue;

				var bestt = tgt.getBestCameraToTarget().inverse();
				Transform2d best = new Transform2d(
						bestt.getX(), bestt.getY(), bestt.getRotation().getZ());
				Transform2d pose = taglocs[id].mul(best.mul(camlocs[ci]));

				var altt = tgt.getAlternateCameraToTarget().inverse();
				Transform2d alt = new Transform2d(
						altt.getX(), altt.getY(), altt.getRotation().getZ());
				Transform2d altpose = taglocs[id].mul(alt.mul(camlocs[ci]));

				if (tgt.getPoseAmbiguity() < 0.03
						|| abs(Util.normRot(pose.theta - odom.now().theta)) < Constants.VISION_THETA_THRESHOLD) {
					// mod 32; overwrites first estimate if ovf; 99% not needed or used
					poses[ptr % 32] = pose;
					times[ptr % 32] = time;
					areas[ptr % 32] = tgt.getArea();
					ptr++;
				} else if (abs(Util.normRot(altpose.theta - odom.now().theta)) < Constants.VISION_THETA_THRESHOLD) {
					poses[ptr % 32] = altpose;
					times[ptr % 32] = time;
					areas[ptr % 32] = tgt.getArea();
					ptr++;
				}
			}
		}
	}

	double last_eval_time = Timer.getFPGATimestamp();

	public void evaluate() { // assume all tags are correct and then unweighted avg
		last_eval_time = Timer.getFPGATimestamp();
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
		double norm = sqrt(tsin * tsin + tcos * tcos);
		Transform2d avg = new Transform2d(tx / totarea, ty / totarea, tcos / norm, tsin / norm);
		if (sz > 3) odom.update_origin(avg);
	}

	@Override
	public void periodic() {
		for (int i = 0; i < cameras.length; i++) update(i);
		evaluate();
	}
}
