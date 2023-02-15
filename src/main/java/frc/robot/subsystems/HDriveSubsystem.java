package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class HDriveSubsystem extends SubsystemBase {
	public ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

	static HDriveSubsystem hdrive;

	TalonFX lef = new TalonFX(DrivetrainConstants.PORT_L1);
	TalonFX lef1 = new TalonFX(DrivetrainConstants.PORT_L2);
	CANSparkMax mid = new CANSparkMax(DrivetrainConstants.PORT_M1, MotorType.kBrushless);
	CANSparkMax mid1 = new CANSparkMax(DrivetrainConstants.PORT_M2, MotorType.kBrushless);
	TalonFX rgt = new TalonFX(DrivetrainConstants.PORT_R1);
	TalonFX rgt1 = new TalonFX(DrivetrainConstants.PORT_R2);

	double vl, vr, vm;
	double vx, vy, w;

	public HDriveSubsystem() {
		super();

		lef1.follow(lef);
		mid1.follow(mid);
		rgt1.follow(rgt);

		dttab.add(this);
		dttab.addNumber("le", () -> getLeftPos());
		dttab.addNumber("me", () -> getMid());
		dttab.addNumber("re", () -> getRightPos());

		dttab.addNumber("vl", () -> vl);
		dttab.addNumber("vr", () -> vr);
		dttab.addNumber("vm", () -> vm);

		dttab.addNumber("vx", () -> vx);
		dttab.addNumber("vy", () -> vy);
		dttab.addNumber("w", () -> w);
	}

	public void drive(double vx, double vy, double w) {
		drive(vx, vy, w, true);
	}

	public void drive(double vx, double vy, double w, boolean preserve) { // TODO directions
		this.vx = vx;
		this.vy = vy;
		this.w = w;
		vl = -vx + w * DrivetrainConstants.RADIUS;
		vr = -vx - w * DrivetrainConstants.RADIUS;
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

		lef.set(ControlMode.PercentOutput, vl * DrivetrainConstants.FWD_RAD_PER_METER / Constants.FALCON_MAX_SPEED);
		rgt.set(ControlMode.PercentOutput, -vr * DrivetrainConstants.FWD_RAD_PER_METER / Constants.FALCON_MAX_SPEED);
		mid.set(vm * DrivetrainConstants.H_RAD_PER_METER / Constants.NEO_MAX_SPEED);
	}

	public double getRightPos() {
		return -rgt.getSelectedSensorPosition() / 2048. * 2 * Math.PI / DrivetrainConstants.FWD_RAD_PER_METER;
	}

	public double getLeftPos() {
		return lef.getSelectedSensorPosition() / 2048. * 2 * Math.PI / DrivetrainConstants.FWD_RAD_PER_METER;
	}

	public double getMid() {
		return mid.getEncoder().getPosition() * 2 * Math.PI / DrivetrainConstants.H_RAD_PER_METER;
	}

	public static HDriveSubsystem getInstance() {
		if (hdrive == null) {
			hdrive = new HDriveSubsystem();
		}
		return hdrive;
	}
}
