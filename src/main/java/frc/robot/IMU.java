package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IMU {
        public AHRS ahrs = new AHRS();
        ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");

        public IMU() {
                ahrs.zeroYaw();
                imutab.addNumber("roll", () -> getRoll()*180/Math.PI);
                imutab.addNumber("pitch", () -> getPitch()*180/Math.PI);
                imutab.addNumber("yaw", () -> getYaw()*180/Math.PI);
                imutab.addNumber("angle", () -> getAngle()*180/Math.PI);
        }

        public boolean isCalibrating() { return ahrs.isCalibrating(); }
        public double getPitch() { return ahrs.getPitch() * Math.PI/180; }
        public double getRoll() { return ahrs.getRoll() * Math.PI/180; }
        public double getYaw() { return -(ahrs.getYaw() * Math.PI/180) % Math.PI; }
        public double getAngle() { return -ahrs.getAngle() * Math.PI/180; }
}