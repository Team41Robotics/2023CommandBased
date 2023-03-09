package frc.robot.commands;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Util;

public class MovArm extends CommandBase {
	double st;
	double vx, vy, t;
	ArmSubsystem arm = ArmSubsystem.getInstance();

	public MovArm(double vx, double vy, double t) {
		addRequirements(arm);

		this.vx = vx;
		this.vy = vy;
		this.t = t;
	}

	@Override
	public void initialize() {
		st = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		double alpha = arm.getJoint1Pos();
		double theta = ELEV_THETA;
		double sec = 1 / cos(alpha - theta);
		double v = sec * (vx * cos(alpha) + vy * sin(alpha));
		double omega = sec / ARM_LEN * (-vx * sin(theta) + vy * cos(theta));

		double max = 0;
		if (abs(v) > ELEV_MAX_SPEED) max = max(max, abs(v) / ELEV_MAX_SPEED);
		if (abs(omega) > JOINT1_MAX_SPEED) max = max(max, abs(omega) / JOINT1_MAX_SPEED);
		if (abs(omega) > JOINT2_MAX_SPEED) max = max(max, abs(omega) / JOINT2_MAX_SPEED);

		if (max > 1) {
			v /= max;
			omega /= max;
		}
		arm.set(v, omega, -omega);
	}

	@Override
	public void end(boolean interrupted) {
		arm.set(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		double alpha = arm.getJoint1Pos();
		double theta = ELEV_THETA;
		double sec = 1 / cos(alpha - theta);
		double v = sec * (vx * cos(alpha) + vy * sin(alpha));
		if (arm.isBotLimitSwitch() && v < 0) return true;
		if (arm.isTopLimitSwitch() && v > 0) return true;
		return Timer.getFPGATimestamp() > st + t
				|| Util.normRot(arm.getJoint1Pos() + Math.PI / 2 - ELEV_THETA) < 5 / 180. * Math.PI;
	}
}
