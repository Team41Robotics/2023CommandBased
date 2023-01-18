package frc.robot;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// NOTE: A+B = BA;
// NOTE2: BE CAREFUL WITH ROTATION WRAPPING !!!!!!

public class Robot extends TimedRobot {
        HDrive hdrive = new HDrive();
        ShuffleboardTab camtab = Shuffleboard.getTab("Camera");
        ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");
        ShuffleboardTab auttab = Shuffleboard.getTab("Auton");

        Odom odom = new Odom();
        PhotonCamera cam = new PhotonCamera("TopCamera");
        public static AHRS imu = new AHRS();

        HashMap<Integer, Transform2d> taglocs = new HashMap<>();

        @Override
        public void robotInit() {
                taglocs.put(1, new Transform2d(new Translation2d(0, 0), new Rotation2d(0)));
                taglocs.put(2, new Transform2d(new Translation2d(0, -1.75), new Rotation2d(0)));

                camtab.addNumber("cam_last_timestamp", () -> last_time);
                camtab.addNumber("latency", () -> ping);
                camtab.addDouble("apriltags_x", () -> apriltags_pose.getX());
                camtab.addDouble("apriltags_y", () -> apriltags_pose.getY());
                camtab.addDouble("apriltags_theta", () -> apriltags_pose.getRotation().getDegrees());

                imutab.addBoolean("is_calibrating",() -> imu.isCalibrating());
                imutab.addNumber("roll", () -> imu.getRoll());
                imutab.addNumber("pitch", () -> imu.getPitch());
                imutab.addNumber("yaw", () -> imu.getYaw());

                auttab.addNumber("X", () -> aut_x);
                auttab.addNumber("Y", () -> aut_y);
                auttab.addNumber("w", () -> aut_omega);
        }

        Transform2d robot_pose = new Transform2d();
        Transform2d apriltags_pose = new Transform2d();
        double last_time = 0;
        double ping = 0;
        public void processAprilTags() {
                PhotonPipelineResult res = cam.getLatestResult();
                double time = res.getTimestampSeconds();
                if(time <= last_time) return;
                last_time = time;

                ping = res.getLatencyMillis();
                double rot = 0;
                Translation2d transl = new Translation2d();
                double totAmbiguity = 0;
                if(res.hasTargets()) {
                        for (PhotonTrackedTarget target: res.getTargets()) {
                                int id = target.getFiducialId();
                                Transform2d trans = Util.transformThreeToTwo(target.getBestCameraToTarget());
                                double ambiguity = target.getPoseAmbiguity() + 0.001;
                                if(taglocs.containsKey(target.getFiducialId())) {
                                        Transform2d pose = taglocs.get(id).plus(trans.inverse());
                                        rot += pose.getRotation().getRadians() * 1/ambiguity;
                                        transl=transl.plus(pose.getTranslation().times(1/ambiguity));
                                        totAmbiguity += 1/ambiguity;
                                }
                        }
                        rot /= totAmbiguity;
                        transl = transl.div(totAmbiguity);
                        apriltags_pose = new Transform2d(transl,Rotation2d.fromRadians(rot));
                }
        }

        @Override
        public void robotPeriodic() {
                processAprilTags();
                odom.robotPeriodic();
                robot_pose = apriltags_pose.plus(odom.delta(last_time));
        }

        double target_angle = 180; // FIXME change // should be fixed now (?)
        double target_x = 2;
        double target_y = -1;
        double aut_x = 0;
        double aut_y = 0;
        double aut_omega = 0;
        @Override
        public void teleopPeriodic() {
                //hdrive.teleopPeriodic();
                double vx, vy, omega;
                aut_x=vx = robot_pose.getX() - target_x;
                aut_y=vy = robot_pose.getY() - target_y;
                vx *= 1.5;
                vy *= 1.5;
                double delta = robot_pose.getRotation().getDegrees() - target_angle;
                delta = Util.normRot(delta * Math.PI/180) * 180/Math.PI;
                System.out.println("delta = " + delta);
                aut_omega = omega = -delta / 45;
                if(DS.getRTrig())hdrive.FODdrive(vx, vy, 0);
                else if(DS.right_js.getRawButton(2))hdrive.FODdrive(0, 0, omega);
                //else if(DS.getRTrig())hdrive.FODdrive(0, 0, omega);
                else hdrive.teleopPeriodic();
        }
}