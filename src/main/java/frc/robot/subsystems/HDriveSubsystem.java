package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.SendableDouble;

public class HDriveSubsystem extends SubsystemBase {
	public ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

	static HDriveSubsystem hdrive;

	TalonFX lef = new TalonFX(DrivetrainConstants.PORT_L1);
	TalonFX lef1 = new TalonFX(DrivetrainConstants.PORT_L2);
	CANSparkMax mid = new CANSparkMax(DrivetrainConstants.PORT_M1, MotorType.kBrushless);
	CANSparkMax mid1 = new CANSparkMax(DrivetrainConstants.PORT_M2, MotorType.kBrushless);
	TalonFX rgt = new TalonFX(DrivetrainConstants.PORT_R1);
	TalonFX rgt1 = new TalonFX(DrivetrainConstants.PORT_R2);

	PIDController lpid = new PIDController(.5, 0, 0);
	PIDController mpid = new PIDController(.5, 0, 0);
	PIDController rpid = new PIDController(.5, 0, 0);

	SendableDouble lff = new SendableDouble(1.5);
	SendableDouble mff = new SendableDouble(1.5);
	SendableDouble rff = new SendableDouble(1.5);

	double vl, vr, vm;
	double lo, mo, ro;

	public HDriveSubsystem() {
		super();

		lef1.follow(lef);
		mid1.follow(mid);
		rgt1.follow(rgt);

		lef.configFactoryDefault();
		lef1.configFactoryDefault();
		rgt.configFactoryDefault();
		rgt1.configFactoryDefault();
		mid.restoreFactoryDefaults();
		mid1.restoreFactoryDefaults();

		lef.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		lef.configVelocityMeasurementWindow(8);
		mid.getEncoder().setMeasurementPeriod(10);
		mid.getEncoder().setAverageDepth(8);
		rgt.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		rgt.configVelocityMeasurementWindow(8);

		dttab.add(this);
		dttab.addNumber("l Encoder", () -> getLeftPos());
		dttab.addNumber("m Encoder", () -> getMidPos());
		dttab.addNumber("r Encoder", () -> getRightPos());
		dttab.add("l", lpid);
		dttab.add("m", mpid);
		dttab.add("r", rpid);

		dttab.add("lff", lff);
		dttab.add("mff", mff);
		dttab.add("rff", rff);

		dttab.addNumber("l PID err", () -> lpid.getPositionError());
		dttab.addNumber("m PID err", () -> mpid.getPositionError());
		dttab.addNumber("r PID err", () -> rpid.getPositionError());

		dttab.addNumber("l Encoder Velocity", () -> getLeftVel());
		dttab.addNumber("m Encoder Velocity", () -> getMidVel());
		dttab.addNumber("r Encoder Velocity", () -> getRightVel());

		dttab.addNumber("l motor setpoint", () -> vl);
		dttab.addNumber("r motor setpoint", () -> vr);
		dttab.addNumber("m motor setpoint", () -> vm);

		dttab.addNumber("l motor output", () -> lo);
		dttab.addNumber("m motor output", () -> mo);
		dttab.addNumber("r motor output", () -> ro);
	}

	public void drive(double vx, double vy, double w) {
		drive(vx, vy, w, true);
	}

	public void drive(double vx, double vy, double w, boolean preserve) {
		vl = vx - w * DrivetrainConstants.RADIUS;
		vr = vx + w * DrivetrainConstants.RADIUS;
		vm = vy;

		double max = 0;
		if (max < Math.abs(vl * DrivetrainConstants.LEFT_SPEED_TO_ONE))
			max = Math.abs(vl * DrivetrainConstants.LEFT_SPEED_TO_ONE);
		if (max < Math.abs(vr * DrivetrainConstants.RIGHT_SPEED_TO_ONE))
			max = Math.abs(vr * DrivetrainConstants.RIGHT_SPEED_TO_ONE);
		if (max < Math.abs(vm * DrivetrainConstants.H_SPEED_TO_ONE * 8)) // TODO remove l8r
		max = Math.abs(vm * DrivetrainConstants.H_SPEED_TO_ONE * 8);

		if (max > 1) {
			vx /= max;
			vy /= max;
			w /= max;
			vl = -vx + w;
			vr = -vx - w;
			vm = vy;
		}
	}

	@Override
	public void periodic() {
		if (DriverStation.isEnabled()) {
			setLeft(vl);
			setMid(vm);
			setRight(vr);
		} else {
			lpid.reset();
			mpid.reset();
			rpid.reset();
		}
	}

	public void setLeft(double vel) {
		lo = lff.x * vel + lpid.calculate(getLeftVel(), vel);
		lef.set(ControlMode.PercentOutput, -lo * DrivetrainConstants.LEFT_SPEED_TO_ONE);
	}

	public void setRight(double vel) {
		ro = rff.x * vel + rpid.calculate(getRightVel(), vel);
		rgt.set(ControlMode.PercentOutput, ro * DrivetrainConstants.RIGHT_SPEED_TO_ONE);
	}

	public void setMid(double vel) {
		mo = mff.x * vel + mpid.calculate(getMidVel(), vel);
		mid.set(-mo * DrivetrainConstants.H_SPEED_TO_ONE);
	}

	public double getRightPos() {
		return rgt.getSelectedSensorPosition() / 2048. * 2 * Math.PI / DrivetrainConstants.RIGHT_RAD_PER_METER;
	}

	public double getLeftPos() {
		return -lef.getSelectedSensorPosition() / 2048. * 2 * Math.PI / DrivetrainConstants.LEFT_RAD_PER_METER;
	}

	public double getMidPos() {
		return -mid.getEncoder().getPosition() * 2 * Math.PI / DrivetrainConstants.H_RAD_PER_METER;
	}

	public double getRightVel() {
		return rgt.getSelectedSensorVelocity() * 10 / 2048. * 2 * Math.PI / DrivetrainConstants.RIGHT_RAD_PER_METER;
	}

	public double getLeftVel() {
		return -lef.getSelectedSensorVelocity() * 10 / 2048. * 2 * Math.PI / DrivetrainConstants.LEFT_RAD_PER_METER;
	}

	public double getMidVel() {
		return mid.getEncoder().getVelocity() / 60. * 2 * Math.PI / DrivetrainConstants.H_RAD_PER_METER;
	}

	public static HDriveSubsystem getInstance() {
		if (hdrive == null) {
			hdrive = new HDriveSubsystem();
		}
		return hdrive;
	}
}
