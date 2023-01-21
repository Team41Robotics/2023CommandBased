package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// NOTE: A+B = BA;
// NOTE2: BE CAREFUL WITH ROTATION WRAPPING !!!!!!

public class Robot extends TimedRobot {
        HDrive hdrive = new HDrive();
        ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");
        ShuffleboardTab auttab = Shuffleboard.getTab("Auton");

        Odom odom = new Odom();
        //PhotonApriltagCameras cams = new PhotonApriltagCameras(new PhotonCamera[]{
                //new PhotonCamera("TopCamera")});
        public static IMU imu = new IMU();
        public static double yawOffset = 0;

        @Override
        public void robotInit() {
                imu.zeroYaw();
                imutab.addBoolean("is_calibrating",() -> imu.isCalibrating());
                imutab.addNumber("roll", () -> imu.getRoll());
                imutab.addNumber("pitch", () -> imu.getPitch());
                imutab.addNumber("yaw", () -> imu.getYaw());

                auttab.addNumber("X", () -> aut_x);
                auttab.addNumber("Y", () -> aut_y);
                auttab.addNumber("theta", () -> Util.normRot(imu.getYaw()) * 180/Math.PI);
                auttab.addNumber("w", () -> aut_w);
        }

        Transform2d robot_pose = new Transform2d();

        @Override
        public void robotPeriodic() {
                //cams.processAprilTags();
                odom.robotPeriodic();
                robot_pose = odom.now();
                //robot_pose = cams.apriltags_pose.plus(odom.delta(cams.last_time));
                //robot_pose = cams.apriltags_pose;
                //if(cams.new_time) {
                        //imu.setYaw(robot_pose.getRotation().getRadians());
                //}
        }

        double tgt_angle = 0; // MATH.PI;
        double tgt_x = 0;//2;
        double tgt_y = 0;//-1;
        double aut_x = 0;
        double aut_y = 0;
        double aut_w = 0;
        PIDController xPID = new PIDController(-5, 0, 0, 0.2);
        PIDController yPID = new PIDController(-5, 0, 0, 0.8);
        PIDController wPID = new PIDController(-0.1, 0, 0, 0.1);
        @Override
        public void teleopPeriodic() {
                double vx, vy, w;
                double theta = imu.getYaw();
                aut_x = vx = xPID.calculate(robot_pose.getX() - tgt_x);
                aut_y = vy = yPID.calculate(robot_pose.getY() - tgt_y);
                aut_w = w = wPID.calculate(Util.normRot(theta - tgt_angle));
                if (DS.getLTrig() || DS.getRTrig()) {
                        if (!DS.getLTrig()) {
                                vx = 0;
                                vy = 0;
                        }
                        if (!DS.getRTrig())
                                w = 0;
                        hdrive.FODdrive(vx, vy, w);
                } else
                        hdrive.teleopPeriodic();
        }
}