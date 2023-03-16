package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmTo;
import frc.robot.commands.ZeroArm;
import frc.robot.util.ArmPosition;
import java.util.Map;

public class ArmSubsystem extends SubsystemBase {
	static ArmSubsystem arm;

	ShuffleboardTab armtab = Shuffleboard.getTab("Arm");

	public CANSparkMax elev = new CANSparkMax(ELEV_ID, MotorType.kBrushless);
	public CANSparkMax elev1 = new CANSparkMax(ELEV1_ID, MotorType.kBrushless);
	public CANSparkMax jt1 = new CANSparkMax(JOINT1_ID, MotorType.kBrushless);
	public CANSparkMax jt2 = new CANSparkMax(JOINT2_ID, MotorType.kBrushless);

	public DigitalInput lower_limit1 = new DigitalInput(LOWERLIMIT1_ID);
	public DigitalInput lower_limit2 = new DigitalInput(LOWERLIMIT2_ID);
	public DigitalInput upper_limit1 = new DigitalInput(UPPERLIMIT1_ID);
	public DigitalInput upper_limit2 = new DigitalInput(UPPERLIMIT2_ID);

	SparkMaxPIDController elev_vpid = elev.getPIDController();
	SparkMaxPIDController elev1_vpid = elev1.getPIDController();
	SparkMaxPIDController jt1_vpid = jt1.getPIDController();
	SparkMaxPIDController jt2_vpid = jt2.getPIDController();

	LEDSubsystem lights = LEDSubsystem.getInstance();
	SendableChooser<ArmPosition> armposes = new SendableChooser<>();
	public Map<String, ArmPosition> positions = Map.of(
			"BALL PICKUP", new ArmPosition(0.000, -0.900, 0.900),
			"BALL TOP", new ArmPosition(1.031, 0.038, 1.203),
			"BALL MID", new ArmPosition(0.833, -0.574, 0.869),
			"ALL  BOT", new ArmPosition(0.116, -0.850, 1.172),
			"CONE MID", new ArmPosition(0.500, 0.183, 0.374),
			"CONE TOP", new ArmPosition(1.040, 0.104, 0.753),
			"CONE PICKUP", new ArmPosition(0.141, 0.095, -0.661),
			"BALL HMN ST", new ArmPosition(0.950, 0.249, 0.55),
			"CONE HMN ST", new ArmPosition(0.950, 0.249, 0.0));

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

		// armtab.getLayout("Arm").withSize(128, 128);

		armtab.addNumber("elev pos", () -> getElevPos());
		armtab.addNumber("joint 1 pos", () -> getJoint1Pos());
		armtab.addNumber("joint 2 pos", () -> getJoint2Pos());
		armtab.addNumber("joint 2 net pos", () -> getJoint1Pos() + getJoint2Pos());
		armtab.addBoolean("top limit switch", this::isTopLimitSwitch);
		armtab.add(this);
	}

	private void createShuffleboardPosition(String name, int y, int x) {
		armtab.add(name, new ArmTo(name)).withPosition(x, y);
	}

	public void init() {
		for (Map.Entry<String, ArmPosition> entry : positions.entrySet()) {
			armposes.addOption(entry.getKey(), entry.getValue());
		}
		createShuffleboardPosition("BALL PICKUP", 1, 1);
		createShuffleboardPosition("BALL TOP", 1, 2);
		createShuffleboardPosition("BALL MID", 1, 3);
		createShuffleboardPosition("BALL HMN ST", 1, 5);

		createShuffleboardPosition("ALL  BOT", 2, 1);

		createShuffleboardPosition("CONE PICKUP", 3, 1);
		createShuffleboardPosition("CONE TOP", 3, 2);
		createShuffleboardPosition("CONE MID", 3, 3);
		createShuffleboardPosition("CONE HMN ST", 3, 5);

		armtab.add("ZERO ARM", new ZeroArm());
		//
		armtab.add(armposes);
		armtab.add("GOTO POS", new ProxyCommand(() -> new ArmTo(armposes.getSelected())));
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

	private boolean jtLock = false;
	private boolean p_upper_limit2 = false;

	@Override
	public void periodic() {
		if (!DriverStation.isEnabled()) {
			if (!upper_limit1.get()) {
				arm.jt2.getEncoder().setPosition(0);
				lights.lf = false;
				lights.left = Color.kGreen;
			}
			if (!p_upper_limit2 && !upper_limit2.get()) { // TODO TODO TODO TODO
				jtLock = !jtLock;
				jt2.setIdleMode((jtLock ? IdleMode.kBrake : IdleMode.kCoast));
				lights.rf = false;
				lights.right = jtLock ? Color.kGreen : Color.kRed;
			}
			p_upper_limit2 = !upper_limit2.get();
		}
		// if (DriverStation.isEnabled() && isTopLimitSwitch())
		// elev.getEncoder().setPosition(ELEV_LEN * ELEV_RAD_PER_METER / 2 / PI);

		// TODO
		// if (elev.getEncoder().getVelocity() > 0 && isTopLimitSwitch()) set(0, 0, 0);
		// if (elev.getEncoder().getVelocity() < 0 && isBotLimitSwitch()) set(0, 0, 0);

		// if (!(getCurrentCommand() instanceof ZeroArm)) {
		// if (jt1.getEncoder().getVelocity() > 0 && getJoint1Pos() > JOINT1_UPPER_BOUND) set(0, 0, 0);
		// if (jt1.getEncoder().getVelocity() < 0 && getJoint1Pos() < JOINT1_LOWER_BOUND) set(0, 0, 0);
		// }
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

	public static ArmSubsystem getInstance() {
		if (arm == null) {
			arm = new ArmSubsystem();
			arm.init();
		}
		return arm;
	}
}
