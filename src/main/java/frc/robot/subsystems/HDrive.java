package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.util.Util.*;
import static java.lang.Math.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.GoTo;

public class HDrive extends SubsystemBase { // TODO sense wheel current if touching ground
	WPI_TalonFX lef = new WPI_TalonFX(PORT_L1);
	WPI_TalonFX lef1 = new WPI_TalonFX(PORT_L2);
	CANSparkMax mid = new CANSparkMax(PORT_M1, MotorType.kBrushless);
	CANSparkMax mid1 = new CANSparkMax(PORT_M2, MotorType.kBrushless);
	WPI_TalonFX rgt = new WPI_TalonFX(PORT_R1);
	WPI_TalonFX rgt1 = new WPI_TalonFX(PORT_R2);

	PIDController lpid = new PIDController(.5, 0, 0);
	PIDController mpid = new PIDController(.5, 0, 0);
	PIDController rpid = new PIDController(.5, 0, 0);

	double lff = 1.5; // TODO sysid
	double mff = 1.5;
	double rff = 1.5;

	double dvl, dvr, dvm;
	double vl, vr, vm;
	double lo, mo, ro;

	public void init() {
		lef.configFactoryDefault();
		lef1.configFactoryDefault();
		rgt.configFactoryDefault();
		rgt1.configFactoryDefault();
		mid.restoreFactoryDefaults();
		mid1.restoreFactoryDefaults();

		lef1.follow(lef);
		mid1.follow(mid);
		rgt1.follow(rgt);

		lef.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		lef.configVelocityMeasurementWindow(8);
		mid.getEncoder().setMeasurementPeriod(10);
		mid.getEncoder().setAverageDepth(8);
		rgt.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		rgt.configVelocityMeasurementWindow(8);
	}

	public void initShuffleboard() {
		ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

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
		dvl = vx - w * RADIUS;
		dvr = vx + w * RADIUS;
		dvm = vy;

		double max = 1;
		if (max < abs(dvl * LEFT_SPEED_TO_ONE * 4)) max = abs(dvl * LEFT_SPEED_TO_ONE * 4);
		if (max < abs(dvr * RIGHT_SPEED_TO_ONE * 4)) max = abs(dvr * RIGHT_SPEED_TO_ONE * 4);
		if (max < abs(dvm * H_SPEED_TO_ONE * 4)) max = abs(dvm * H_SPEED_TO_ONE * 4);

		if (preserve) {
			dvl /= max;
			dvr /= max;
			dvm /= max;
		}
	}

	@Override
	public void periodic() {
		if (DriverStation.isEnabled()) { // TODO refactor ramp times out
			// TODO make this cleaner too
			if (true || getCurrentCommand() instanceof GoTo) {
				vm = abs(dvm - vm) < LOOP_TIME * 1 || abs(dvm) < abs(vm) ? dvm : vm + signum(dvm - vm) * LOOP_TIME * 1;
			} else vm = dvm;
			vl = dvl;
			vr = dvr;
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
}
