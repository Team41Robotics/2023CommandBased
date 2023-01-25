package frc.robot.subsystems;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Matrix2d;

public class PhotonVisionSubsystem extends SubsystemBase {
        /*
        static PhotonVisionSubsystem pv;

        PhotonCamera cam = new PhotonCamera("TopCamera");

        HashMap<Integer, Matrix2d> taglocs = new HashMap<>();
        Matrix2d cam_to_com = new Matrix2d(0, -0.12, 0);

        public PhotonVisionSubsystem() {
                // OUTDATED
                taglocs.put(1, new Matrix2d(0,0,0));
                taglocs.put(2, new Matrix2d(0,-1.75,0));
        }

        double last_time = Timer.getFPGATimestamp();
        @Override
        public void periodic() {
                PhotonPipelineResult res = cam.getLatestResult();
                if(res.getTimestampSeconds() > last_time) {
                        last_time = res.getTimestampSeconds();
                        for (PhotonTrackedTarget tgt : res.getTargets()) {
                                int id = tgt.getFiducialId();
                                if(taglocs.containsKey(id)) {
                                        Matrix2d best = new Matrix2d(tgt.getBestCameraToTarget().inverse());
                                        Matrix2d alt = new Matrix2d(tgt.getAlternateCameraToTarget().inverse());
                                        double ambig = tgt.getPoseAmbiguity();
                                        if(ambig > 0.2) {
                                                Matrix2d camera_pose = best.mul(taglocs.get(id));
                                                Matrix2d com_pose = cam_to_com.mul(camera_pose);
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
        */
}