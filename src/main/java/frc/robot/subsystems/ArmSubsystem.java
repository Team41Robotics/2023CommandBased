package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
	CANSparkMax elev = new CANSparkMax(ELEV_ID, MotorType.kBrushless);
	CANSparkMax joint1 = new CANSparkMax(JOINT1_ID, MotorType.kBrushless);
	CANSparkMax joint2 = new CANSparkMax(JOINT2_ID, MotorType.kBrushless);

	SparkMaxPIDController elev_vpid = elev.getPIDController();
	SparkMaxPIDController joint1_vpid = joint1.getPIDController();
	SparkMaxPIDController joint2_vpid = joint2.getPIDController();

	public ArmSubsystem() {
		zero();
		setPID(elev_vpid, 1, 0, 0, 0);
		setPID(joint1_vpid, 1, 0, 0, 0);
		setPID(joint2_vpid, 1, 0, 0, 0);
	}

	private void setPID(SparkMaxPIDController pid, double kP, double kI, double kD, double kIz) {
		pid.setP(kP);
		pid.setI(kI);
		pid.setD(kD);
		pid.setIZone(kIz);
	}

	public void zero() {
		elev.getEncoder().setPosition(ELEV_PACKAGED_POSITION / 2 / PI / ELEV_METERS_PER_RAD);
		joint1.getEncoder().setPosition(JOINT1_PACKAGED_POSITION / 2 / PI / JOINT1_RATIO);
		joint2.getEncoder().setPosition(JOINT2_PACKAGED_POSITION / 2 / PI / JOINT2_RATIO);

		elev.restoreFactoryDefaults();
		joint1.restoreFactoryDefaults();
		joint2.restoreFactoryDefaults();
	}
	// TODO dynamic zeroing of encoder based on limit switches on elevator

	public void set(double elev_vel, double joint1_vel, double joint2_vel) {
		set(elev_vel, joint1_vel, joint2_vel, 0, 0, 0);
	}

	public void set(
			double elev_vel,
			double joint1_vel,
			double joint2_vel,
			double elev_acc,
			double joint1_acc,
			double joint2_acc) {
		elev_vpid.setReference(
				elev_vel / ELEV_METERS_PER_RAD / 2 / PI,
				ControlType.kVelocity,
				0,
				ELEV_kG + ELEV_kS * signum(elev_vel) + ELEV_kV * elev_vel + ELEV_kA * elev_acc,
				ArbFFUnits.kVoltage);
		joint1_vpid.setReference(
				joint1_vel * JOINT1_RATIO / 2 / PI,
				ControlType.kVelocity,
				0,
				JOINT1_kG * cos(getJoint1Pos())
						+ JOINT1_kS * signum(joint1_vel)
						+ JOINT1_kV * joint1_vel
						+ JOINT1_kA * joint1_acc,
				ArbFFUnits.kVoltage);
		joint2_vpid.setReference(
				joint2_vel * JOINT2_RATIO / 2 / PI,
				ControlType.kVelocity,
				0,
				JOINT2_kG * cos(getJoint2Pos())
						+ JOINT2_kS * signum(joint2_vel)
						+ JOINT2_kV * joint2_vel
						+ JOINT2_kA * joint2_acc,
				ArbFFUnits.kVoltage);
	}

	@Override
	public void periodic() {
		// TODO dynamic zeroing of encoder based on limit switches on elevator
	}

	public double getElevPos() {
		return elev.getEncoder().getPosition() * 2 * PI * ELEV_METERS_PER_RAD;
	}

	public double getJoint1Pos() {
		return joint1.getEncoder().getPosition() * 2 * PI / JOINT1_RATIO;
	}

	public double getJoint2Pos() {
		return joint2.getEncoder().getPosition() * 2 * PI / JOINT2_RATIO;
	}
}
