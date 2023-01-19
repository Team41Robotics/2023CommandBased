package frc.robot;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PhotonApriltagCameras {
        HashMap<Integer, Transform2d> taglocs = new HashMap<>();

        PhotonCamera[] cams;
        ShuffleboardTab camtab = Shuffleboard.getTab("Camera");

        double last_time = 0;
        double ping = 0;
        Transform2d apriltags_pose = new Transform2d();
        boolean new_time = false;

        PhotonApriltagCameras(PhotonCamera[] cams) {
                taglocs.put(1, new Transform2d(new Translation2d(0, 0), new Rotation2d(0)));
                taglocs.put(2, new Transform2d(new Translation2d(0, -1.75), new Rotation2d(0)));

                this.cams = cams;

                camtab.addNumber("cam_last_timestamp", () -> last_time);
                camtab.addNumber("latency", () -> ping);
                camtab.addDouble("apriltags_x", () -> apriltags_pose.getX());
                camtab.addDouble("apriltags_y", () -> apriltags_pose.getY());
                camtab.addDouble("apriltags_theta", () -> apriltags_pose.getRotation().getDegrees());
        }

        public void processAprilTags() {
                new_time = false;
                double time = 0;
                double rot = 0;
                Translation2d transl = new Translation2d();
                double totAmbiguity = 0;
                for (PhotonCamera cam : cams) {
                        PhotonPipelineResult res = cam.getLatestResult();
                        ping = res.getLatencyMillis();
                        if (res.hasTargets()) {
                                for (PhotonTrackedTarget target : res.getTargets()) {
                                        int id = target.getFiducialId();
                                        Transform2d trans = Util.transformThreeToTwo(target.getBestCameraToTarget());
                                        double ambiguity = target.getPoseAmbiguity() + 0.001;
                                        if (taglocs.containsKey(target.getFiducialId())) {
                                                Transform2d pose = taglocs.get(id).plus(trans.inverse());
                                                rot += pose.getRotation().getRadians() * 1 / ambiguity;
                                                transl = transl.plus(pose.getTranslation().times(1 / ambiguity));
                                                totAmbiguity += 1 / ambiguity;
                                        }
                                }
                        }
                }
                if (totAmbiguity > 0) {
                        new_time = true;
                        last_time = time / totAmbiguity;
                        rot /= totAmbiguity;
                        transl = transl.div(totAmbiguity);
                        apriltags_pose = new Transform2d(transl, Rotation2d.fromRadians(rot));
                }
        }
}