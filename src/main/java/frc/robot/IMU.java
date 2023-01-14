package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IMU {
        double cx = 0;
        double cy = 0;
        double cz = 0;
        double pt = Timer.getFPGATimestamp();

        double cal_starttime;
        double cal_n = 0;
        double cal_x = 0;
        double cal_y = 0;
        double cal_z = 0;

        double ax = 0;
        double ay = 0;
        double az = 0;
        double cal_ax = 0;
        double cal_ay = 0;
        double cal_az = 0;

        AHRS ahrs = new AHRS(); // future note: may turn update rate up to 200Hz

        ShuffleboardTab gyrotab = Shuffleboard.getTab("Gyro");
        ShuffleboardTab acceltab = Shuffleboard.getTab("Accelerometer");

        double[] arr = new double[100];
        int ptr = 0;
        double angle = 0;
        public synchronized void init() {
                calibrate();
                gyrotab.addBoolean("is_calibrating",() -> ahrs.isCalibrating());
                //gyrotab.addNumber("raw_X", () -> ahrs.getRawGyroX());
                //gyrotab.addNumber("raw_Y", () -> ahrs.getRawGyroY());
                //gyrotab.addNumber("raw_Z", () -> ahrs.getRawGyroZ());

                gyrotab.addNumber("internal_roll", () -> ahrs.getRoll());
                gyrotab.addNumber("internal_pitch", () -> ahrs.getPitch());
                gyrotab.addNumber("internal_yaw", () -> ahrs.getYaw());

                acceltab.addNumber("raw_X", () -> (ahrs.getRawAccelX() - cal_ax));
                acceltab.addNumber("raw_Y", () -> (ahrs.getRawAccelY() - cal_ay-1));
                acceltab.addNumber("raw_Z", () -> (ahrs.getRawAccelZ() - cal_az));

                acceltab.addNumber("angle", () -> {
                        double ang = 180/Math.PI*Math.acos(-Math.max(-1,ahrs.getRawAccelY()-cal_ay-1));
                        arr[ptr++] = ang;
                        ptr %= 100;
                        double sum = 0;
                        for(double x:arr)sum+=x;
                        angle = sum/100;
                        return sum/100;
                });

                //gyrotab.addNumber("x", () -> (cx%360));
                //gyrotab.addNumber("y", () -> (cy%360));
                //gyrotab.addNumber("z", () -> (cz%360));
        }

        public synchronized void periodic() {
                double t = Timer.getFPGATimestamp();
                double dt = t - pt;
                cx += (ahrs.getRawGyroX()-cal_x) * dt;
                cy += (ahrs.getRawGyroY()-cal_y) * dt;
                cz += (ahrs.getRawGyroZ()-cal_z) * dt;

                pt = t;

                if (ahrs.isCalibrating()) cal_starttime = t;
                if (isCalibrating()) {
                        cal_x += ahrs.getRawGyroX();
                        cal_y += ahrs.getRawGyroY();
                        cal_z += ahrs.getRawGyroZ();
                        cal_ax += ahrs.getRawAccelX();
                        cal_ay += ahrs.getRawAccelY();
                        cal_az += ahrs.getRawAccelZ();
                        cal_n++;
                        cx=cy=cz = 0;
                }
                else {
                        cal_x /= cal_n;
                        cal_y /= cal_n;
                        cal_z /= cal_n;
                        cal_ax /= cal_n;
                        cal_ay /= cal_n;
                        cal_az /= cal_n;
                        cal_n = 1;
                }
        }

        public synchronized void calibrate() {
                cal_starttime = Timer.getFPGATimestamp();
        }

        public synchronized boolean isCalibrating() {
                return Timer.getFPGATimestamp() < cal_starttime + 5;
        }

        public synchronized void zeroX() { cx = 0; }
        public synchronized void zeroY() { cy = 0; }
        public synchronized void zeroZ() { cz = 0; }

        public synchronized double getX() { return cx; }
        public synchronized double getY() { return cy; }
        public synchronized double getZ() { return cz; }
}
