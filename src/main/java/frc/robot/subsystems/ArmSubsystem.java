package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
	static ArmSubsystem arm;

	ShuffleboardTab armtab = Shuffleboard.getTab("Arm");

	public CANSparkMax elev = new CANSparkMax(ELEV_ID, MotorType.kBrushless);
	public CANSparkMax elev1 = new CANSparkMax(ELEV1_ID, MotorType.kBrushless);
	public CANSparkMax jt1 = new CANSparkMax(JOINT1_ID, MotorType.kBrushless);
	public CANSparkMax jt2 = new CANSparkMax(JOINT2_ID, MotorType.kBrushless);

	SparkMaxPIDController elev_vpid = elev.getPIDController();
	SparkMaxPIDController elev1_vpid = elev1.getPIDController();
	SparkMaxPIDController jt1_vpid = jt1.getPIDController();
	SparkMaxPIDController jt2_vpid = jt2.getPIDController();

	public ArmSubsystem() {
		elev.restoreFactoryDefaults();
		elev1.restoreFactoryDefaults();
		jt1.restoreFactoryDefaults();
		jt2.restoreFactoryDefaults();
		elev.setIdleMode(IdleMode.kBrake);
		elev1.setIdleMode(IdleMode.kBrake);
		jt1.setIdleMode(IdleMode.kBrake);
		jt2.setIdleMode(IdleMode.kCoast);

		setPID(elev_vpid, 0, 0, 0, 0);
		setPID(elev1_vpid, 0, 0, 0, 0);
		setPID(jt1_vpid, 0, 0, 0, 0);
		setPID(jt2_vpid, 0, 0, 0, 0);

		armtab.addNumber("elev pos", () -> getElevPos());
		armtab.addNumber("joint 1 pos", () -> getJoint1Pos());
		armtab.addNumber("joint 2 pos", () -> getJoint2Pos());
		armtab.addNumber("joint 2 net pos", () -> getJoint1Pos() + getJoint2Pos());
		armtab.add(this);
	}

	private void setPID(SparkMaxPIDController pid, double kP, double kI, double kD, double kIz) {
		pid.setP(kP);
		pid.setI(kI);
		pid.setD(kD);
		pid.setIZone(kIz);
	}

	private void setMotor(SparkMaxPIDController pid, double vel, double ff) {
		pid.setReference(vel, ControlType.kVelocity, 0, ff, ArbFFUnits.kVoltage);
	}

	public void set(double elev_vel, double jt1_vel, double jt2_vel) {
		set(elev_vel, jt1_vel, jt2_vel, 0, 0, 0);
	}

	public void set(double elev_v, double jt1_v, double jt2_v, double elev_a, double jt1_a, double jt2_a) {
		// TODO kG probably varies
		setMotor(
				elev_vpid,
				elev_v * ELEV_RAD_PER_METER / 2 / PI,
				ELEV_kG + ELEV_kS * signum(elev_v) + ELEV_kV * elev_v + ELEV_kA * elev_a);
		setMotor(
				elev1_vpid,
				elev_v * ELEV_RAD_PER_METER / 2 / PI,
				ELEV_kG + ELEV_kS * signum(elev_v) + ELEV_kV * elev_v + ELEV_kA * elev_a);
		setMotor(
				jt1_vpid,
				jt1_v * JOINT1_RATIO / 2 / PI,
				JOINT1_kG * cos(getJoint1Pos()) + JOINT1_kS * signum(jt1_v) + JOINT1_kV * jt1_v + JOINT1_kA * jt1_a);
		setMotor(
				jt2_vpid,
				jt2_v * JOINT2_RATIO / 2 / PI,
				JOINT2_kG * cos(getJoint1Pos() + getJoint2Pos())
						+ JOINT2_kS * signum(jt2_v)
						+ JOINT2_kV * jt2_v
						+ JOINT2_kA * jt2_a);
	}

	@Override
	public void periodic() {
                double ve=0,v1=0,v2=0;
                if(Robot.DS.getRawButton(6)) ve=.8;
                else if(Robot.DS.getRawButton(5)) ve=-.5;
                if(Robot.DS.getRawButton(1)) v1=.3;
                else if(Robot.DS.getRawButton(2)) v1=-.3;
                if(Robot.rightjs.getPOV() == 0) v2=.5;
                else if(Robot.rightjs.getPOV() == 180) v2=-.5;
                set(ve,v1,v2);
        }

	public double getElevPos() {
		return elev.getEncoder().getPosition() * 2 * PI / ELEV_RAD_PER_METER;
	}

	public double getJoint1Pos() {
		return jt1.getEncoder().getPosition() * 2 * PI / JOINT1_RATIO;
	}

	public double getJoint2Pos() {
		return jt2.getEncoder().getPosition() * 2 * PI / JOINT2_RATIO;
	}

	public static ArmSubsystem getInstance() {
		if (arm == null) {
			arm = new ArmSubsystem();
		}
		return arm;
	}
}
