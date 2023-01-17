package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Robot extends TimedRobot {
        HDrive hdrive = new HDrive();
        ShuffleboardTab camtab = Shuffleboard.getTab("Camera");
        ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");

        PhotonCamera cam = new PhotonCamera("TopCamera");

        public static AHRS imu = new AHRS();

        HashMap<Integer, Transform2d> taglocs = new HashMap<>();
        Field2d field = new Field2d();

        @Override
        public void robotInit() {
                taglocs.put(1, new Transform2d(new Translation2d(0, 0), new Rotation2d(0)));
                taglocs.put(2, new Transform2d(new Translation2d(0, -1.75), new Rotation2d(0)));

                camtab.addNumber("cam_last_timestamp", () -> last_time);
                camtab.addNumber("latency", () -> ping);
                camtab.addInteger("id", () -> target_ids.get(0));
                camtab.addIntegerArray("target_ids", () -> {
                        long[] x = new long[target_ids.size()];
                        int p=0;
                        for (int id : target_ids) x[p++]=id;
                        return x;
                });
                camtab.addDoubleArray("target_x", () -> {
                        double[] x = new double[target_loc.size()];
                        int p=0;
                        for (Transform2d l : target_loc) x[p++] = l.getX();
                        return x;
                });
                camtab.addDoubleArray("target_y", () -> {
                        double[] x = new double[target_loc.size()];
                        int p=0;
                        for (Transform2d l : target_loc) x[p++] = l.getY();
                        return x;
                });
                camtab.addDoubleArray("pose_x", () -> {
                        double[] x = new double[target_loc.size()];
                        int p=0;
                        for (Transform2d l : predicted_pose) x[p++] = l.getX();
                        return x;
                });
                camtab.addDoubleArray("pose_y", () -> {
                        double[] x = new double[target_loc.size()];
                        int p=0;
                        for (Transform2d l : predicted_pose) x[p++] = l.getY();
                        return x;
                });
                camtab.addDoubleArray("pose_theta", () -> {
                        double[] x = new double[target_loc.size()];
                        int p=0;
                        for (Transform2d l : predicted_pose) x[p++] = l.getRotation().getDegrees();
                        return x;
                });
                camtab.add("field",field);
                imutab.addBoolean("is_calibrating",() -> imu.isCalibrating());
                imutab.addNumber("roll", () -> imu.getRoll());
                imutab.addNumber("pitch", () -> imu.getPitch());
                imutab.addNumber("yaw", () -> imu.getYaw());

                imutab.addNumber("X", () -> (imu.getRawAccelX()));
                imutab.addNumber("Y", () -> (imu.getRawAccelY()));
                imutab.addNumber("Z", () -> (imu.getRawAccelZ()));
        }

        ArrayList<Integer> target_ids = new ArrayList<>();
        ArrayList<Transform2d> target_loc = new ArrayList<>(); // cam to tag
        ArrayList<Transform2d> predicted_pose = new ArrayList<>(); // field to cam
        double last_time = 0;
        double ping = 0;
        public void processAprilTags() {
                PhotonPipelineResult res = cam.getLatestResult();
                double time = res.getTimestampSeconds();
                if(time <= last_time) return;
                ping = res.getLatencyMillis();
                target_ids.clear(); target_loc.clear(); predicted_pose.clear();
                if(res.hasTargets()) {
                        for (PhotonTrackedTarget target: res.getTargets()) {
                                target_ids.add(target.getFiducialId());
                                Transform2d trans = Util.transformThreeToTwo(target.getBestCameraToTarget());
                                target_loc.add(trans);
                                // let A,B be transforms
                                // A+B = BA // wpilib be weird ikr and you can't do reflections either
                                // we need to find Tcam_tag Ttag_field
                                // = Ttag_field + Tcam_tag
                                if(taglocs.containsKey(target.getFiducialId()))
                                        predicted_pose.add(
                                                taglocs.get(target.getFiducialId()).plus(trans.inverse())
                                        );
                        }
                }
                if(predicted_pose.size()>0)
                        field.setRobotPose(new Pose2d(predicted_pose.get(0).getTranslation(),
                                predicted_pose.get(0).getRotation()));
                last_time = time;
        }

        @Override
        public void robotPeriodic() {
                processAprilTags();
        }

        @Override
        public void teleopPeriodic() {
                hdrive.teleopPeriodic();
        }
}