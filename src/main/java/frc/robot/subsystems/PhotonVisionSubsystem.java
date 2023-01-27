package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Transform2d;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
        static PhotonVisionSubsystem pv;

        PhotonCamera cam = new PhotonCamera("TopCamera");

        HashMap<Integer, Transform2d> taglocs = new HashMap<>();
        Transform2d cam_to_com = new Transform2d(0, -0.12, 0);

        public PhotonVisionSubsystem() {
                // OUTDATED
                taglocs.put(1, new Transform2d(0, 4, 0));
                taglocs.put(2, new Transform2d(0, 3.25, 0));
        }

        double last_time = Timer.getFPGATimestamp();

        @Override
        public void periodic() {
                PhotonPipelineResult res = cam.getLatestResult();
                if (res.getTimestampSeconds() > last_time) {
                        last_time = res.getTimestampSeconds();
                        for (PhotonTrackedTarget tgt : res.getTargets()) {
                                int id = tgt.getFiducialId();
                                if (taglocs.containsKey(id)) {
                                        var bestt = tgt.getBestCameraToTarget().inverse();
                                        var altt = tgt.getAlternateCameraToTarget().inverse();
                                        Transform2d best =
                                                        new Transform2d(bestt.getX(), bestt.getY(), bestt.getRotation().getZ());
                                        Transform2d alt = new Transform2d(altt.getX(), altt.getY(), altt.getRotation().getZ());
                                        double ambig = tgt.getPoseAmbiguity();
                                        if (ambig > 0.2) {
                                                Transform2d camera_pose = best.mul(taglocs.get(id));
                                                Transform2d com_pose = cam_to_com.mul(camera_pose);
                                                OdomSubsystem.getInstance().update_from(com_pose, last_time);
                                        }
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
