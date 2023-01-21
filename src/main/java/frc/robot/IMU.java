package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class IMU {
        public AHRS ahrs = new AHRS();
        public double yawOffset = 0;

        public boolean isCalibrating() { return ahrs.isCalibrating(); }
        public double getPitch() { return ahrs.getPitch() * Math.PI/180; }
        public double getRoll() { return ahrs.getRoll() * Math.PI/180; }
        public double getYaw() { return ahrs.getYaw() * Math.PI/180; }
        public double getAngle() { return ahrs.getAngle() * Math.PI/180; }

        public void setYaw(double yaw) { yawOffset = yaw - getYaw(); }
        public void zeroYaw() { setYaw(0); }
}
