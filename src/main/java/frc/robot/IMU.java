package frc.robot;

import static java.lang.Math.PI;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IMU {
	private AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 66);
	ShuffleboardTab imutab = Shuffleboard.getTab("Inertial");
	double yawOffset = 0;

	public IMU() {
		imutab.addNumber("roll", () -> getRoll() * 180 / PI);
		imutab.addNumber("pitch", () -> getPitch() * 180 / PI);
		imutab.addNumber("yaw", () -> getYaw() * 180 / PI);
		imutab.addNumber("angle", () -> getAngle() * 180 / PI);
	}

	public boolean isCalibrating() {
		return ahrs.isCalibrating();
	}

	public void zeroYaw() {
		yawOffset = -ahrs.getYaw();
	}

	public double getPitch() {
		return ahrs.getPitch() * PI / 180;
	}

	public double getRoll() {
		return ahrs.getRoll() * PI / 180;
	}

	public double getYaw() {
		return (-ahrs.getYaw() - yawOffset) * PI / 180;
	}

	public double getAngle() {
		return (-ahrs.getAngle() - yawOffset) * PI / 180;
	}
}
