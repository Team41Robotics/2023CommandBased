package frc.robot.subsystems;

import static java.lang.Math.PI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
	CANSparkMax elev = new CANSparkMax(ArmConstants.ELEV_ID, MotorType.kBrushless);
	CANSparkMax joint1 = new CANSparkMax(ArmConstants.JOINT1_ID, MotorType.kBrushless);
	CANSparkMax joint2 = new CANSparkMax(ArmConstants.JOINT2_ID, MotorType.kBrushless);

	SparkMaxPIDController elev_vpid = elev.getPIDController();
	SparkMaxPIDController joint1_vpid = joint1.getPIDController();
	SparkMaxPIDController joint2_vpid = joint2.getPIDController();

	PIDController elev_ppid = new PIDController(1, 0, 0);
	PIDController joint1_ppid = new PIDController(1, 0, 0);
	PIDController joint2_ppid = new PIDController(1, 0, 0);

        TrapezoidProfile elev_prof;
        TrapezoidProfile joint1_prof;
        TrapezoidProfile joint2_prof;
	ArmPosition setpoint;

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
		elev.getEncoder().setPosition(ArmConstants.ELEV_PACKAGED_POSITION / 2 / PI / ArmConstants.ELEV_METERS_PER_RAD);
		joint1.getEncoder().setPosition(ArmConstants.JOINT1_PACKAGED_POSITION / 2 / PI / ArmConstants.JOINT1_RATIO);
		joint2.getEncoder().setPosition(ArmConstants.JOINT2_PACKAGED_POSITION / 2 / PI / ArmConstants.JOINT2_RATIO);

		elev.restoreFactoryDefaults();
		joint1.restoreFactoryDefaults();
		joint2.restoreFactoryDefaults();
	}
	// TODO dynamic zeroing of encoder based on limit switches on elevator

	public void set(ArmPosition pos) {
		this.setpoint = pos;
	}

	@Override
	public void periodic() {
		// TODO dynamic zeroing of encoder based on limit switches on elevator
	}

	public double getElevPos() {
		return elev.getEncoder().getPosition() * 2 * PI * ArmConstants.ELEV_METERS_PER_RAD;
	}

	public double getJoint1Pos() {
		return joint1.getEncoder().getPosition() * 2 * PI / ArmConstants.JOINT1_RATIO;
	}

	public double getJoint2Pos() {
		return joint2.getEncoder().getPosition() * 2 * PI / ArmConstants.JOINT2_RATIO;
	}
}
