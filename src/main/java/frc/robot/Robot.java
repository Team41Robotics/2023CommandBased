package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// NOTE: A+B = BA;
// NOTE2: BE CAREFUL WITH ROTATION WRAPPING !!!!!!

public class Robot extends TimedRobot {
        HDrive hdrive;
        ShuffleboardTab auttab = Shuffleboard.getTab("Auton");

        public static IMU imu;
        Odom odom;
        //PhotonApriltagCameras cams = new PhotonApriltagCameras(new PhotonCamera[]{
                //new PhotonCamera("TopCamera")});

        @Override
        public void robotInit() {
                hdrive = new HDrive();
                imu = new IMU();
                odom = new Odom();
                auttab.addNumber("X", () -> aut_x);
                auttab.addNumber("Y", () -> aut_y);
                auttab.addNumber("theta", () -> Util.normRot(imu.getYaw()) * 180/Math.PI);
                auttab.addNumber("w", () -> aut_w);
                auttab.add("wPID", wPID);
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

        Transform2d target = new Transform2d(new Translation2d(0,0), new Rotation2d(0));
        Transform2d target1 = new Transform2d(new Translation2d(0,0), new Rotation2d(0));
        Transform2d target2 = new Transform2d(new Translation2d(1,1), new Rotation2d(0));
        double aut_x = 0;
        double aut_y = 0;
        double aut_w = 0;
        PIDController xPID = new PIDController(-5, 0, 0, 0.2);
        PIDController yPID = new PIDController(5, 0, 0, 0.8);
        PIDController wPID = new PIDController(-1, 0, 0, 0.1);
        @Override
        public void teleopPeriodic() {
                double vx, vy, w;
                //double theta = imu.getYaw();
                if(DS.getLTrig()) target = target1;
                if(DS.getRTrig()) target = target2;
                Transform2d tgt_in_robot = target.plus(robot_pose.inverse());
                aut_x = vx = xPID.calculate(tgt_in_robot.getX());
                aut_y = vy = yPID.calculate(tgt_in_robot.getY());
                aut_w = w = wPID.calculate(Util.normRot(tgt_in_robot.getRotation().getRadians()));
                if (DS.getLTrig() || DS.getRTrig()) {
                        hdrive.drive(vx, vy, w);
                } else
                        hdrive.teleopPeriodic();

        }
}