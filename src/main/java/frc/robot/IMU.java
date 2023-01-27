package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IMU {
  private AHRS ahrs = new AHRS();
  ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");
  double yawOffset = 0;

  public IMU() {
    imutab.addNumber("roll", () -> getRoll() * 180 / Math.PI);
    imutab.addNumber("pitch", () -> getPitch() * 180 / Math.PI);
    imutab.addNumber("yaw", () -> getYaw() * 180 / Math.PI);
    imutab.addNumber("angle", () -> getAngle() * 180 / Math.PI);
  }

  public boolean isCalibrating() {
    return ahrs.isCalibrating();
  }

  public void zeroYaw() {
    yawOffset = -ahrs.getYaw();
  }

  public double getPitch() {
    return ahrs.getPitch() * Math.PI / 180;
  }

  public double getRoll() {
    return ahrs.getRoll() * Math.PI / 180;
  }

  public double getYaw() {
    return ((-ahrs.getYaw() - yawOffset) * Math.PI / 180);
  }

  public double getAngle() {
    return (-ahrs.getAngle() - yawOffset) * Math.PI / 180;
  }
}
