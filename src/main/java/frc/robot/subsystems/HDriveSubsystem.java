package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import static java.lang.Math.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HDriveSubsystem extends SubsystemBase { // TODO sense wheel current if touching ground
	public ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

	static HDriveSubsystem hdrive;

	WPI_TalonFX lef = new WPI_TalonFX(PORT_L1);
	WPI_TalonFX lef1 = new WPI_TalonFX(PORT_L2);
	CANSparkMax mid = new CANSparkMax(PORT_M1, MotorType.kBrushless);
	CANSparkMax mid1 = new CANSparkMax(PORT_M2, MotorType.kBrushless);
	WPI_TalonFX rgt = new WPI_TalonFX(PORT_R1);
	WPI_TalonFX rgt1 = new WPI_TalonFX(PORT_R2);

	PIDController lpid = new PIDController(.5, 0, 0);
	PIDController mpid = new PIDController(.5, 0, 0);
	PIDController rpid = new PIDController(.5, 0, 0);

	double lff = 1.5;
	double mff = 1.5;
	double rff = 1.5;

	double vl, vr, vm;
	double lo, mo, ro;

	private void configureMotors() {
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
	}

	public HDriveSubsystem() {
		super();

		configureMotors();

		dttab.add(this);
		dttab.addNumber("l Encoder", () -> getLeftPos());
		dttab.addNumber("m Encoder", () -> getMidPos());
		dttab.addNumber("r Encoder", () -> getRightPos());
		dttab.add("l", lpid);
		dttab.add("m", mpid);
		dttab.add("r", rpid);

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

	public void drive(double vx, double vy, double w, boolean preserve) { // TODO add accel
		vl = vx - w * RADIUS;
		vr = vx + w * RADIUS;
		vm = vy;

		double max = 0;
		if (max < abs(vl * LEFT_SPEED_TO_ONE)) max = abs(vl * LEFT_SPEED_TO_ONE);
		if (max < abs(vr * RIGHT_SPEED_TO_ONE)) max = abs(vr * RIGHT_SPEED_TO_ONE);
		if (max < abs(vm * H_SPEED_TO_ONE)) max = abs(vm * H_SPEED_TO_ONE);

		if (preserve && max > 1) {
			vx /= max;
			vy /= max;
			w /= max;
			vl = vx - w * RADIUS;
			vr = vx + w * RADIUS;
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

	public void setLeft(double vel) { // TODO when SYSID
		lo = lff * vel + lpid.calculate(getLeftVel(), vel);
		lef.set(-lo * LEFT_SPEED_TO_ONE);
	}

	public void setRight(double vel) {
		ro = rff * vel + rpid.calculate(getRightVel(), vel);
		rgt.set(ro * RIGHT_SPEED_TO_ONE);
	}

	public void setMid(double vel) {
		mo = mff * vel + mpid.calculate(getMidVel(), vel);
		mid.set(-mo * H_SPEED_TO_ONE);
	}

	private double talonToRad(TalonFX talon) {
		return talon.getSelectedSensorPosition() / 2048. * 2 * PI;
	}

	private double talonToRadPerSecond(TalonFX talon) {
		return talon.getSelectedSensorVelocity() * 10 / 2048. * 2 * PI;
	}

	private double neoToRad(CANSparkMax neo) {
		return neo.getEncoder().getPosition() * 2 * PI;
	}

	private double neoToRadPerSecond(CANSparkMax neo) {
		return neo.getEncoder().getPosition() * 2 * PI / 60.;
	}

	public double getRightPos() {
		return talonToRad(rgt) / RIGHT_RAD_PER_METER;
	}

	public double getLeftPos() {
		return -talonToRad(lef) / LEFT_RAD_PER_METER;
	}

	public double getMidPos() {
		return -neoToRad(mid) / H_RAD_PER_METER;
	}

	public double getRightVel() {
		return talonToRadPerSecond(rgt) / RIGHT_RAD_PER_METER;
	}

	public double getLeftVel() {
		return -talonToRadPerSecond(lef) / LEFT_RAD_PER_METER;
	}

	public double getMidVel() {
		return -neoToRadPerSecond(mid) / H_RAD_PER_METER;
	}

	public static HDriveSubsystem getInstance() {
		if (hdrive == null) {
			hdrive = new HDriveSubsystem();
		}
		return hdrive;
	}
}
