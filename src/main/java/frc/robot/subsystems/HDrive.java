package frc.robot.subsystems;

import static frc.robot.constants.Constants.*;
import static frc.robot.constants.MechanicalConstants.DrivetrainConstants.*;
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

public class HDrive extends SubsystemBase { // TODO sense wheel current if touching ground
	final WPI_TalonFX lef = new WPI_TalonFX(Ports.CAN_DT_L1);
	final WPI_TalonFX lef1 = new WPI_TalonFX(Ports.CAN_DT_L2);
	final CANSparkMax mid = new CANSparkMax(Ports.CAN_DT_M1, MotorType.kBrushless);
	final CANSparkMax mid1 = new CANSparkMax(Ports.CAN_DT_M2, MotorType.kBrushless);
	final WPI_TalonFX rgt = new WPI_TalonFX(Ports.CAN_DT_R1);
	final WPI_TalonFX rgt1 = new WPI_TalonFX(Ports.CAN_DT_R2);

	final PIDController lpid = new PIDController(.5, 0, 0);
	final PIDController mpid = new PIDController(.5, 0, 0);
	final PIDController rpid = new PIDController(.5, 0, 0);

	double dvl, dvr, dvm;
	double vl, vr, vm;

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
	}

	public void drive(double vx, double vy, double w) {
		drive(vx, vy, w, 1);
	}

	public double renormalize(double vx, double vy, double w, double preserve) {
		double max = 1;
		max = max(max, abs(dvl / FWD_CONSTRAINTS.maxVelocity * preserve));
		max = max(max, abs(dvr / FWD_CONSTRAINTS.maxVelocity * preserve));
		max = max(max, abs(dvm / MID_CONSTRAINTS.maxVelocity * preserve));
		return max;
	}

	public void drive(double vx, double vy, double w, double preserve) {
		dvl = vx - w * RADIUS;
		dvr = vx + w * RADIUS;
		dvm = vy;

		double max = renormalize(vx, vy, w, preserve);

		dvl /= max;
		dvr /= max;
		dvm /= max;
	}

	@Override
	public void periodic() {
		if (DriverStation.isEnabled()) {
			if (abs(dvm - vm) > LOOP_TIME * MID_CONSTRAINTS.maxAcceleration) {
				vm += signum(dvm - vm) * LOOP_TIME * MID_CONSTRAINTS.maxAcceleration;
			} else vm = dvm;
			if (abs(dvl - vl) > LOOP_TIME * FWD_CONSTRAINTS.maxAcceleration) {
				vl += signum(dvl - vl) * LOOP_TIME * FWD_CONSTRAINTS.maxAcceleration;
			} else vl = dvl;
			if (abs(dvr - vr) > LOOP_TIME * FWD_CONSTRAINTS.maxAcceleration) {
				vr += signum(dvr - vr) * LOOP_TIME * FWD_CONSTRAINTS.maxAcceleration;
			} else vr = dvr;

			setLeft(vl, 0);
			setMid(vm, 0); // TODO does this even work properly
			setRight(vr, 0);
		} else {
			lpid.reset();
			mpid.reset();
			rpid.reset();
		}
	}

	public void setLeft(double vel, double acc) {
		lef.setVoltage(-FWD_IDENTF.getOutput(0, vel, acc));
	}

	public void setRight(double vel, double acc) {
		rgt.setVoltage(FWD_IDENTF.getOutput(0, vel, acc));
	}

	public void setMid(double vel, double acc) {
		mid.setVoltage(-MID_IDENTF.getOutput(0, vel, acc));
	}

	public double getRightPos() {
		return talonToRad(rgt) * FWD_METER_PER_RAD;
	}

	public double getLeftPos() {
		return -talonToRad(lef) * FWD_METER_PER_RAD;
	}

	public double getMidPos() {
		return -neoToRad(mid) * H_METER_PER_RAD;
	}

	public double getRightVel() {
		return talonToRadPerSecond(rgt) * FWD_METER_PER_RAD;
	}

	public double getLeftVel() {
		return -talonToRadPerSecond(lef) * FWD_METER_PER_RAD;
	}

	public double getMidVel() {
		return -neoToRadPerSecond(mid) * H_METER_PER_RAD;
	}
}
