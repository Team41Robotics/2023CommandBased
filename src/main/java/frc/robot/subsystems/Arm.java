package frc.robot.subsystems;

import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Constants.ArmPos.*;
import static frc.robot.constants.MechanicalConstants.ArmConstants.*;
import static java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmTo;
import frc.robot.constants.Constants.Ports;

public class Arm extends SubsystemBase {
	public final CANSparkMax elev = new CANSparkMax(Ports.CAN_ELEV, MotorType.kBrushless);
	public final CANSparkMax elev1 = new CANSparkMax(Ports.CAN_ELEV1, MotorType.kBrushless);
	public final CANSparkMax jt1 = new CANSparkMax(Ports.CAN_JOINT1, MotorType.kBrushless);
	public final CANSparkMax jt2 = new CANSparkMax(Ports.CAN_JOINT2, MotorType.kBrushless);

	// TODO plug limit switches into spark max?
	final DigitalInput lower_limit1 = new DigitalInput(Ports.DIO_LOWERLIMIT1);
	final DigitalInput lower_limit2 = new DigitalInput(Ports.DIO_LOWERLIMIT2);
	final DigitalInput upper_limit1 = new DigitalInput(Ports.DIO_UPPERLIMIT1);
	final DigitalInput upper_limit2 = new DigitalInput(Ports.DIO_UPPERLIMIT2);

	public final DigitalInput joint2_limit = new DigitalInput(Ports.DIO_JOINT2_LIMIT);

	final SparkMaxPIDController elev_vpid = elev.getPIDController();
	final SparkMaxPIDController elev1_vpid = elev1.getPIDController();
	final SparkMaxPIDController jt1_vpid = jt1.getPIDController();
	final SparkMaxPIDController jt2_vpid = jt2.getPIDController();

	public final PIDController elev_pid = new PIDController(4, 0, 0); // TODO ppid tuning
	public final PIDController jt1_pid = new PIDController(3, 0, 0);
	public final PIDController jt2_pid = new PIDController(5, 0, 0);

	final SendableChooser<ArmPos> armposes = new SendableChooser<>();

	public void init() {
		elev.restoreFactoryDefaults();
		elev1.restoreFactoryDefaults();
		jt1.restoreFactoryDefaults();
		jt2.restoreFactoryDefaults();
		elev.setIdleMode(IdleMode.kBrake);
		elev1.setIdleMode(IdleMode.kBrake);
		jt1.setIdleMode(IdleMode.kBrake);
		jt2.setIdleMode(IdleMode.kCoast);
		elev.getEncoder().setPosition(0);
		jt2.setInverted(true);

		setPID(elev_vpid, 0, 0, 0, 0); // TODO actually have vpids (use LQR gains?)
		setPID(elev1_vpid, 0, 0, 0, 0);
		setPID(jt1_vpid, 0, 0, 0, 0);
		setPID(jt2_vpid, 0, 0, 0, 0);
		// TODO maybe config vel meas filter
	}

	public void initShuffleboard() {
		ShuffleboardTab armtab = Shuffleboard.getTab("Arm");

		armtab.add(this);
		armtab.addNumber("elev pos", () -> getElevPos());
		armtab.addNumber("joint 1 pos", () -> getJoint1Pos());
		armtab.addNumber("joint 2 pos", () -> getJoint2Pos());
		armtab.addNumber("joint 2 net pos", () -> getJoint1Pos() + getJoint2Pos());
		armtab.addBoolean("top limit switch", this::isTopLimitSwitch);
		armtab.addBoolean("joint2 limit switch", () -> !joint2_limit.get());

		for (ArmPos pos : ArmPos.values()) {
			armposes.addOption(pos.name(), pos);
		}
		createShuffleboardPosition(BALL_PICKUP, 1, 1);
		createShuffleboardPosition(BALL_TOP, 1, 2);
		createShuffleboardPosition(BALL_MID, 1, 3);
		createShuffleboardPosition(BALL_PLATFORM, 1, 5);
		createShuffleboardPosition(CONE_SLIDE, 1, 6);

		createShuffleboardPosition(ALL_BOT, 2, 1);

		createShuffleboardPosition(CONE_PICKUP, 3, 1);
		createShuffleboardPosition(CONE_TOP, 3, 2);
		createShuffleboardPosition(CONE_MID, 3, 3);
		createShuffleboardPosition(CONE_PLATFORM, 3, 5);

		armtab.add(armposes);
		armtab.add("GOTO POS", new ProxyCommand(() -> new ArmTo(armposes.getSelected())));
	}

	private void createShuffleboardPosition(ArmPos pos, int y, int x) {
		Shuffleboard.getTab("arm").add(pos.name(), new ArmTo(pos)).withPosition(x, y);
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

	public void hold() {
		set(getElevPos(), getJoint1Pos(), getJoint2Pos(), 0, 0, 0);
	}

	public void set(double elev_pos, double jt1_pos, double jt2_pos, double elev_vel, double jt1_vel, double jt2_vel) {
		set(elev_pos, jt1_pos, jt2_pos, elev_vel, jt1_vel, jt2_vel, 0, 0, 0);
	}

	public void set(
			double elev_pos,
			double jt1_pos,
			double jt2_pos,
			double elev_v,
			double jt1_v,
			double jt2_v,
			double elev_a,
			double jt1_a,
			double jt2_a) {
		if (abs(elev_pos - getElevPos()) > 0.1) elev_pid.setI(0);
		else elev_pid.setI(1);
		double elev_fb = elev_pid.calculate(getElevPos(), elev_pos);
		double jt1_fb = jt1_pid.calculate(getJoint1Pos(), jt1_pos);
		double jt2_fb = jt2_pid.calculate(getJoint2Pos(), jt2_pos);

		double jt1_cos = cos(getJoint1Pos());
		double jt2_cos = sin(getJoint2Pos() + getJoint1Pos());

		setMotor(elev_vpid, elev_v * ELEV_RAD_PER_METER / 2 / PI, ELEV_IDENTF.getOutput(1, elev_v + elev_fb, elev_a));
		setMotor(elev1_vpid, elev_v * ELEV_RAD_PER_METER / 2 / PI, ELEV_IDENTF.getOutput(1, elev_v + elev_fb, elev_a));
		setMotor(jt1_vpid, jt1_v * JOINT1_RATIO / 2 / PI, JOINT1_IDENTF.getOutput(jt1_cos, jt1_v + jt1_fb, jt1_a));
		setMotor(jt2_vpid, jt2_v * JOINT2_RATIO / 2 / PI, JOINT2_IDENTF.getOutput(jt2_cos, jt2_v + jt2_fb, jt2_a));
		// setMotor(jt2_vpid, 0, 4);
	}

	@Override
	public void periodic() {
		// TODO automatic zeroing at start
		if (isTopLimitSwitch()) elev.getEncoder().setPosition(ELEV_LEN * ELEV_RAD_PER_METER / 2 / PI);
		if (isBotLimitSwitch()) elev.getEncoder().setPosition(0);

		// TODO limits (use sparkmax builtins?)
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

	public boolean isBotLimitSwitch() {
		return !(lower_limit1.get() && lower_limit2.get());
	}

	public boolean isTopLimitSwitch() {
		return !(upper_limit1.get() && upper_limit2.get());
	}
}
