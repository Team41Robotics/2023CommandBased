package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Robot;

public class HDriveSubsystem extends SubsystemBase {
	public ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

	static HDriveSubsystem hdrive;

	TalonFX lef = new TalonFX(DrivetrainConstants.PORT_L1);
	TalonFX lef1 = new TalonFX(DrivetrainConstants.PORT_L2);
	CANSparkMax mid = new CANSparkMax(DrivetrainConstants.PORT_M1, MotorType.kBrushless);
	CANSparkMax mid1 = new CANSparkMax(DrivetrainConstants.PORT_M2, MotorType.kBrushless);
	TalonFX rgt = new TalonFX(DrivetrainConstants.PORT_R1);
	TalonFX rgt1 = new TalonFX(DrivetrainConstants.PORT_R2);

	PIDController lpid = new PIDController(1, 0, 0);
	PIDController mpid = new PIDController(0, 0, 0);
	PIDController rpid = new PIDController(1, 0, 0);

	double vl, vr, vm;

	public HDriveSubsystem() {
		super();

		lef1.follow(lef);
		mid1.follow(mid);
		rgt1.follow(rgt);

		lef.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		lef.configVelocityMeasurementWindow(8);
		lef1.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		lef1.configVelocityMeasurementWindow(8);
		mid.getEncoder().setMeasurementPeriod(10);
		mid.getEncoder().setAverageDepth(8);
		mid1.getEncoder().setMeasurementPeriod(10);
		mid1.getEncoder().setAverageDepth(8);
		rgt.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		rgt.configVelocityMeasurementWindow(8);
		rgt1.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
		rgt1.configVelocityMeasurementWindow(8);

		dttab.add(this);
		dttab.addNumber("le", () -> getLeftPos());
		dttab.addNumber("me", () -> getMidPos());
		dttab.addNumber("re", () -> getRightPos());

		dttab.addNumber("lv", () -> getLeftVel());
		dttab.addNumber("mv", () -> getMidVel());
		dttab.addNumber("rv", () -> getRightVel());

		dttab.addNumber("vl", () -> vl);
		dttab.addNumber("vr", () -> vr);
		dttab.addNumber("vm", () -> vm);
	}

	public void drive(double vx, double vy, double w) {
		drive(vx, vy, w, true);
	}

	public void drive(double vx, double vy, double w, boolean preserve) { // TODO directions (testing)
		vl = -vx + w * DrivetrainConstants.RADIUS;
		vr = vx + w * DrivetrainConstants.RADIUS;
		vm = -vy;

		/*double max = Math.max(Math.max(Math.abs(vl), Math.abs(vm)), Math.abs(vr));
		if (max > 1) { // FIXME FWD &
			vx /= max;
			vy /= max;
			w /= max;
			vl = -vx + w;
			vr = -vx - w;
			vm = vy;
		}*/

		// lef.set(ControlMode.PercentOutput, vl * DrivetrainConstants.FWD_RAD_PER_METER / Constants.FALCON_MAX_SPEED);
		// rgt.set(ControlMode.PercentOutput, vr * DrivetrainConstants.FWD_RAD_PER_METER / Constants.FALCON_MAX_SPEED);
		// mid.set(vm * DrivetrainConstants.H_RAD_PER_METER / Constants.NEO_MAX_SPEED);
	}

	@Override
	public void periodic() {
		if (Robot.r.isEnabled()) {
			ControlMode PO = ControlMode.PercentOutput;
			System.out.println("vl " + lpid.calculate(getLeftVel(), vl));
			System.out.println("vv________________ " + getLeftVel() + " vl " + vl);
			lef.set(PO, vl + lpid.calculate(-getLeftVel(), vl) * DrivetrainConstants.FWD_SPEED_TO_ONE);
			mid.set(vm + mpid.calculate(getMidVel(), vm) * DrivetrainConstants.H_SPEED_TO_ONE);
			rgt.set(PO, vr + rpid.calculate(getRightVel(), vr) * DrivetrainConstants.FWD_SPEED_TO_ONE);
		} else {
			lpid.reset();
			mpid.reset();
			rpid.reset();
		}
	}

	public double getRightPos() {
		return rgt.getSelectedSensorPosition() / 2048. * 2 * Math.PI / DrivetrainConstants.FWD_RAD_PER_METER;
	}

	public double getLeftPos() {
		return -lef.getSelectedSensorPosition() / 2048. * 2 * Math.PI / DrivetrainConstants.FWD_RAD_PER_METER;
	}

	public double getMidPos() {
		return mid.getEncoder().getPosition() * 2 * Math.PI / DrivetrainConstants.H_RAD_PER_METER;
	}

	public double getRightVel() {
		return rgt.getSelectedSensorVelocity() * 10 / 2048. * 2 * Math.PI / DrivetrainConstants.FWD_RAD_PER_METER;
	}

	public double getLeftVel() {
		return -lef.getSelectedSensorVelocity() * 10 / 2048. * 2 * Math.PI / DrivetrainConstants.FWD_RAD_PER_METER;
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
