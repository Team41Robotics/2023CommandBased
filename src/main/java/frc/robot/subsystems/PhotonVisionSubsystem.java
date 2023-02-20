package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Transform2d;
import frc.robot.Util;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase { // TODO a lot
	private static final double THETA_THRESHOLD = 10 / 180. * Math.PI;

	static PhotonVisionSubsystem pv;

	OdomSubsystem odom = OdomSubsystem.getInstance();
	ShuffleboardTab camtab = Shuffleboard.getTab("Camera");

	Transform2d[] taglocs = new Transform2d[] { // CHANGE WITH COORD SYSTEM
		null,
		new Transform2d(15.51310, 1.06341, Math.PI),
		new Transform2d(15.51310, 2.73981, Math.PI),
		new Transform2d(15.51310, 4.41621, Math.PI),
		new Transform2d(16.17832, 6.74161, Math.PI),
		new Transform2d(0.36168, 6.74161, 0),
		new Transform2d(1.02690, 4.41621, 0),
		new Transform2d(1.02690, 2.73981, 0),
		new Transform2d(1.02690, 1.06341, 0),
	};

	PhotonCamera[] cameras = new PhotonCamera[] {new PhotonCamera("TopCamera")};
	Transform2d[] camlocs = new Transform2d[] {new Transform2d(-0.42, 0, 0)}; // FIXME
	double[] last_time = new double[] {Timer.getFPGATimestamp()};

	Transform2d[] poses = new Transform2d[32];
	double[] times = new double[32];
	int ptr = 0;

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

				if (Math.abs(Util.normRot(pose.theta - odom.now().theta)) < THETA_THRESHOLD) {
					// mod 32; overwrites first estimate if ovf; 99% not needed or used
					poses[ptr % 32] = pose;
					times[ptr % 32] = time;
					ptr++;
				} else if (Math.abs(Util.normRot(altpose.theta - odom.now().theta)) < THETA_THRESHOLD) {
					poses[ptr % 32] = altpose;
					times[ptr % 32] = time;
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

		int sz = Math.min(32, ptr);
		if (sz == 0) return;
		for (int i = 0; i < 32 && i < ptr; i++) {
			Transform2d o = odom.origin_if(poses[i], times[i]);
			tx += o.x;
			ty += o.y;
			tsin += o.sin;
			tcos += o.cos;
		}
		ptr = 0;
		double norm = Math.sqrt(tsin * tsin + tcos * tcos);
		Transform2d avg = new Transform2d(tx / sz, ty / sz, tcos / norm, tsin / norm);
		odom.update_origin(avg);
	}

	@Override
	public void periodic() {
		for (int i = 0; i < cameras.length; i++) update(i);
		if (Timer.getFPGATimestamp() > last_eval_time + 0.2) evaluate();
	}

	public static PhotonVisionSubsystem getInstance() {
		if (pv == null) {
			pv = new PhotonVisionSubsystem();
		}
		return pv;
	}
}
