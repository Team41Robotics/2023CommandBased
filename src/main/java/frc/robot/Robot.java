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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Robot extends TimedRobot {
        Joystick left_js = new Joystick(1);
        Joystick right_js = new Joystick(0);
        boolean FOD = false;

        HDrive hdrive = new HDrive();
        ShuffleboardTab camtab = Shuffleboard.getTab("Camera");
        ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");

        PhotonCamera cam = new PhotonCamera("TopCamera");

        AHRS imu = new AHRS();

        HashMap<Integer, Transform2d> taglocs = new HashMap<>();
        Field2d field = new Field2d();

        @Override
        public void robotInit() {
                taglocs.put(1, new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI/2)));
                taglocs.put(2, new Transform2d(new Translation2d(1.75, 0), new Rotation2d(Math.PI/2)));

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
                                Transform2d trans = three_to_two(target.getBestCameraToTarget());
                                target_loc.add(trans);
                                // PLUS is matrix multiplication wtf
                                // in the wrong direction
                                if(taglocs.containsKey(target.getFiducialId()))
                                        predicted_pose.add(trans.inverse().plus(taglocs.get(target.getFiducialId())));
                        }
                }
                if(predicted_pose.size()>0)
                        field.setRobotPose(new Pose2d(predicted_pose.get(0).getTranslation(),
                                predicted_pose.get(0).getRotation()));
                last_time = time;
        }

        public Transform2d three_to_two(Transform3d three) {
                return new Transform2d(three.getTranslation().toTranslation2d(), three.getRotation().toRotation2d());
        }

        @Override
        public void robotPeriodic() {
                processAprilTags();
        }

        @Override
        public void teleopPeriodic() {
                double vf = -deadZone(left_js.getY());
                double vs = -deadZone(left_js.getX());
                double omega = deadZone(right_js.getX());

                if(Math.abs(omega) < 0.5) omega=0;

                double robot_angle = imu.getYaw();
                double vx = vf * Math.cos(robot_angle*Math.PI/180) - vs * Math.sin(robot_angle*Math.PI/180);
                double vy = vf * Math.sin(robot_angle*Math.PI/180) + vs * Math.cos(robot_angle*Math.PI/180);

                if(right_js.getRawButtonPressed(2)) FOD = !FOD;
                if(left_js.getRawButton(2)) imu.zeroYaw();

                if(left_js.getRawButton(1))
                        hdrive.drive(0.5 * Math.signum(imu.getPitch())*Math.sqrt(Math.abs(imu.getPitch() / 15)),0,0);
                else if(FOD) 
                        hdrive.drive(-vx, vy,-omega);
                else
                        hdrive.drive(vf, -vs, -omega);
        }


        private double deadZone(double joystickAxis){
                return ((Math.abs(joystickAxis)>0.2)?joystickAxis:0);
        }
}