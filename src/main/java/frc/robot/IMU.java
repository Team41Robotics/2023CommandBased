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

        AHRS ahrs = new AHRS(); // future note: may turn update rate up to 200Hz

        ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");

        public synchronized void init() {
                calibrate();
                imutab.addBoolean("is_calibrating",() -> ahrs.isCalibrating());
                imutab.addNumber("raw_X", () -> ahrs.getRawGyroX());
                imutab.addNumber("raw_Y", () -> ahrs.getRawGyroY());
                imutab.addNumber("raw_Z", () -> ahrs.getRawGyroZ());

                imutab.addNumber("internal_roll", () -> ahrs.getRoll());
                imutab.addNumber("internal_pitch", () -> ahrs.getPitch());
                imutab.addNumber("internal_yaw", () -> ahrs.getYaw());

                imutab.addNumber("x", () -> cx);
                imutab.addNumber("y", () -> cy);
                imutab.addNumber("z", () -> cz);
        }

        public synchronized void periodic() {
                double t = Timer.getFPGATimestamp();
                double dt = t - pt;
                cx += (ahrs.getRawGyroX()-cal_x) * dt;
                cy += (ahrs.getRawGyroY()-cal_y) * dt;
                cz += (ahrs.getRawGyroZ()-cal_z) * dt;

                pt = t;

                if (isCalibrating()) {
                        cal_x += ahrs.getRawGyroX();
                        cal_y += ahrs.getRawGyroY();
                        cal_z += ahrs.getRawGyroZ();
                        cal_n++;
                        cx=cy=cz = 0;
                }
                else {
                        cal_x /= cal_n;
                        cal_y /= cal_n;
                        cal_z /= cal_n;
                        cal_n = 1;
                }
        }

        public synchronized void calibrate() {
                cal_starttime = Timer.getFPGATimestamp();
        }

        public synchronized boolean isCalibrating() {
                return Timer.getFPGATimestamp() < cal_starttime + 2;
        }

        public synchronized void zeroX() { cx = 0; }
        public synchronized void zeroY() { cy = 0; }
        public synchronized void zeroZ() { cz = 0; }

        public synchronized double getX() { return cx; }
        public synchronized double getY() { return cy; }
        public synchronized double getZ() { return cz; }
}
