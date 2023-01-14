package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
        Joystick left_js = new Joystick(1);
        Joystick right_js = new Joystick(0);
        private boolean FOD;

        HDrive hdrive = new HDrive();
        ShuffleboardTab camtab = Shuffleboard.getTab("Camera");

        //PhotonCamera cam = new PhotonCamera("TopCamera");
        //PhotonPipelineResult res;

        IMU imu = new IMU();

        @Override
        public void robotInit() {
                /*
                camtab.addNumber("cam_last_timestamp", () -> res.getTimestampSeconds());
                camtab.addNumber("latency", () -> res.getLatencyMillis());
                camtab.addBoolean("has_target", () -> res.hasTargets());
                camtab.addIntegerArray("target_ids", () -> {
                        if(res.hasTargets()) {
                                long[] arr = new long[res.getTargets().size()];
                                int i=0;
                                for (PhotonTrackedTarget target: res.getTargets()) {
                                        arr[i++] = (long)target.getFiducialId();
                                }
                                return arr;
                        }
                        else return new long[0];
                });
                camtab.addDoubleArray("target_x", () -> {
                        if(res.hasTargets()) {
                                double[] arr = new double[res.getTargets().size()];
                                int i=0;
                                for (PhotonTrackedTarget target: res.getTargets()) {
                                        arr[i++] = (double)target.getBestCameraToTarget().getX();
                                }
                                return arr;
                        }
                        else return new double[0];
                });
                camtab.addDoubleArray("target_y", () -> {
                        if(res.hasTargets()) {
                                double[] arr = new double[res.getTargets().size()];
                                int i=0;
                                for (PhotonTrackedTarget target: res.getTargets()) {
                                        arr[i++] = (double)target.getBestCameraToTarget().getY();
                                }
                                return arr;
                        }
                        else return new double[0];
                });
                */
                imu.init();
        }

        public void autonomousInit() {
                imu.calibrate();
        }

        @Override
        public void robotPeriodic() {
                //res = cam.getLatestResult();
                imu.periodic();
        }

        @Override
        public void teleopPeriodic() {


                
                double vf = -deadZone(left_js.getY());
                double vs = -deadZone(left_js.getX());
                System.out.println(deadZone(left_js.getY()));
                double omega = deadZone(right_js.getX());
                if(Math.abs(omega) < 0.5) omega=0;
                double robot_angle = imu.ahrs.getYaw();
                double vx = vf * Math.cos(robot_angle*Math.PI/180) - vs * Math.sin(robot_angle*Math.PI/180);
                double vy = vf * Math.sin(robot_angle*Math.PI/180) + vs * Math.cos(robot_angle*Math.PI/180);
                // robot centric drive
                //hdrive.drive(-left_js.getY(), left_js.getX(), -right_js.getX());
                //hdrive.drive(-vx,vy,-omega);
                if(right_js.getRawButtonPressed(2))
                        FOD = !FOD;

                if(left_js.getRawButton(2)) imu.ahrs.zeroYaw();
                if(left_js.getRawButton(1)){
                        hdrive.drive(0.5 * Math.signum(imu.ahrs.getPitch())*Math.sqrt(Math.abs(imu.ahrs.getPitch() / 15)),0,0);
                }else if(FOD) {hdrive.drive(-vx,vy,-omega);}
                else{
                        hdrive.drive(vf, -vs, -omega);
                } 

                //hdrive.drive( left_js.getY() * (left_js.getRawButton(0)? imu.angle/15: 1),0,0);
        }


        private double deadZone(double joystickAxis){
                return ((Math.abs(joystickAxis)>0.2)?joystickAxis:0);
        }
}