package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IMU {
        private AHRS ahrs = new AHRS();
        public double yawOffset = 0;
        ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");

        public IMU() {
                zeroYaw();
                imutab.add("imu", ahrs);
                imutab.addNumber("offset", () -> yawOffset);
                imutab.addNumber("roll", () -> getRoll()*180/Math.PI);
                imutab.addNumber("pitch", () -> getPitch()*180/Math.PI);
                imutab.addNumber("yaw", () -> getYaw()*180/Math.PI);
                imutab.addNumber("angle", () -> getAngle()*180/Math.PI);
        }

        public boolean isCalibrating() { return ahrs.isCalibrating(); }
        public double getPitch() { return ahrs.getPitch() * Math.PI/180; }
        public double getRoll() { return ahrs.getRoll() * Math.PI/180; }
        public double getYaw() { return (ahrs.getYaw() * Math.PI/180 + yawOffset) % Math.PI; }
        public double getAngle() { return ahrs.getAngle() * Math.PI/180 + yawOffset; }

        public void setYaw(double yaw) {
                System.out.println("ZERO YAW: " + yaw);
                yawOffset = yaw - ahrs.getAngle() * Math.PI/180;
                System.out.println("current ahrs yaw: " +
                (ahrs.getYaw()*Math.PI/180) + " yaw offset: " + yawOffset);
        }
        public void zeroYaw() { setYaw(0); }
}
