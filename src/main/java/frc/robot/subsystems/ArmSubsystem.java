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
	CANSparkMax jt1 = new CANSparkMax(JOINT1_ID, MotorType.kBrushless);
	CANSparkMax jt2 = new CANSparkMax(JOINT2_ID, MotorType.kBrushless);

	SparkMaxPIDController elev_vpid = elev.getPIDController();
	SparkMaxPIDController jt1_vpid = jt1.getPIDController();
	SparkMaxPIDController jt2_vpid = jt2.getPIDController();

	public ArmSubsystem() {
		zero();
		setPID(elev_vpid, 1, 0, 0, 0);
		setPID(jt1_vpid, 1, 0, 0, 0);
		setPID(jt2_vpid, 1, 0, 0, 0);
	}

	private void setPID(SparkMaxPIDController pid, double kP, double kI, double kD, double kIz) {
		pid.setP(kP);
		pid.setI(kI);
		pid.setD(kD);
		pid.setIZone(kIz);
	}

	public void zero() {
		elev.getEncoder().setPosition(ELEV_PACKAGED_POSITION / 2 / PI / ELEV_METERS_PER_RAD);
		jt1.getEncoder().setPosition(JOINT1_PACKAGED_POSITION / 2 / PI / JOINT1_RATIO);
		jt2.getEncoder().setPosition(JOINT2_PACKAGED_POSITION / 2 / PI / JOINT2_RATIO);

		elev.restoreFactoryDefaults();
		jt1.restoreFactoryDefaults();
		jt2.restoreFactoryDefaults();
	}
	// TODO dynamic zeroing of encoder based on limit switches on elevator

	public void set(double elev_vel, double jt1_vel, double jt2_vel) {
		set(elev_vel, jt1_vel, jt2_vel, 0, 0, 0);
	}

	public void set(double elev_v, double jt1_v, double jt2_v, double elev_a, double jt1_a, double jt2_a) {
		elev_vpid.setReference(
				elev_v / ELEV_METERS_PER_RAD / 2 / PI,
				ControlType.kVelocity,
				0,
				ELEV_kG + ELEV_kS * signum(elev_v) + ELEV_kV * elev_v + ELEV_kA * elev_a,
				ArbFFUnits.kVoltage);
		jt1_vpid.setReference(
				jt1_v * JOINT1_RATIO / 2 / PI,
				ControlType.kVelocity,
				0,
				JOINT1_kG * cos(getJoint1Pos()) + JOINT1_kS * signum(jt1_v) + JOINT1_kV * jt1_v + JOINT1_kA * jt1_a,
				ArbFFUnits.kVoltage);
		jt2_vpid.setReference(
				jt2_v * JOINT2_RATIO / 2 / PI,
				ControlType.kVelocity,
				0,
				JOINT2_kG * cos(getJoint2Pos()) + JOINT2_kS * signum(jt2_v) + JOINT2_kV * jt2_v + JOINT2_kA * jt2_a,
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
		return jt1.getEncoder().getPosition() * 2 * PI / JOINT1_RATIO;
	}

	public double getJoint2Pos() {
		return jt2.getEncoder().getPosition() * 2 * PI / JOINT2_RATIO;
	}
}
