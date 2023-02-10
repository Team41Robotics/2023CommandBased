package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class HDriveSubsystem extends SubsystemBase {
	public ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

	static HDriveSubsystem hdrive;

	TalonFX bot_lef = new TalonFX(DrivetrainConstants.BOTTOM_LEFT); // TODO MOTOR GROUPS?
	TalonFX top_lef = new TalonFX(DrivetrainConstants.TOP_LEFT);
	CANSparkMax mid = new CANSparkMax(DrivetrainConstants.MID, MotorType.kBrushless);
	TalonFX top_rgt = new TalonFX(DrivetrainConstants.TOP_RIGHT);
	TalonFX bot_rgt = new TalonFX(DrivetrainConstants.BOTTOM_RIGHT);

	double vl, vr, vm;
	double vx, vy, w;

	public HDriveSubsystem() {
		super();

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

	public void drive(double vx, double vy, double w, boolean preserve) {
		this.vx = vx;
		this.vy = vy;
		this.w = w;
		vl = -vx + w * DrivetrainConstants.RADIUS;
		vr = -vx - w * DrivetrainConstants.RADIUS;
		vm = vy;

		double max = Math.max(Math.max(Math.abs(vl), Math.abs(vm)), Math.abs(vr));
		if (false && max > 1) { // FIXME FWD &
			vx /= max;
			vy /= max;
			w /= max;
			vl = -vx + w;
			vr = -vx - w;
			vm = vy;
		}

		// v = omega r = 2 pi r * rpm = 2 pi r * gratio * raw speed
		bot_lef.set(ControlMode.Velocity, vl / DrivetrainConstants.FWD_ROTS_PER_METER);
		top_lef.set(ControlMode.Velocity, vl / DrivetrainConstants.FWD_ROTS_PER_METER);
		top_rgt.set(ControlMode.Velocity, vr / DrivetrainConstants.FWD_ROTS_PER_METER);
		bot_rgt.set(ControlMode.Velocity, vr / DrivetrainConstants.FWD_ROTS_PER_METER);
		mid.set(vm * DrivetrainConstants.H_ROTS_PER_METER);
	}

	public double getRightPos() {
		return bot_rgt.getSelectedSensorPosition() / 2048 / DrivetrainConstants.FWD_ROTS_PER_METER;
	}

	public double getLeftPos() {
		return bot_lef.getSelectedSensorPosition() / 2048 / DrivetrainConstants.FWD_ROTS_PER_METER;
	}

	public double getMid() {
		return mid.getEncoder().getPosition() / 2048 / DrivetrainConstants.H_ROTS_PER_METER;
	}

	public static HDriveSubsystem getInstance() {
		if (hdrive == null) {
			hdrive = new HDriveSubsystem();
		}
		return hdrive;
	}
}
