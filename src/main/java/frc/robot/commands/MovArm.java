package frc.robot.commands;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Util;

public class MovArm extends CommandBase {
	double st;
	double vx, vy, t;
	ArmSubsystem arm = ArmSubsystem.getInstance();

	public MovArm(double vx, double vy, double t) {
		this.vx = vx;
		this.vy = vy;
		this.t = t;
		st = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		double alpha = arm.getJoint1Pos();
		double theta = ArmConstants.ELEV_THETA;
		double sec = 1 / cos(alpha - theta);
		double v = sec * (vx * cos(alpha) + vy * sin(alpha));
		double omega = sec / arm.getElevPos() * (-vx * sin(theta) + vy * cos(theta));

		arm.set(v, omega, -omega);
	}

	@Override
	public void end(boolean interrupted) {
		arm.set(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		double alpha = arm.getJoint1Pos();
		double theta = ArmConstants.ELEV_THETA;
		double sec = 1 / cos(alpha - theta);
		double v = sec * (vx * cos(alpha) + vy * sin(alpha));
		if (v > 0 && arm.isFwdLimitSwitch()) return true;
		if (v < 0 && arm.isRevLimitSwitch()) return true;
		return Timer.getFPGATimestamp() > st + t
				|| Util.normRot(arm.getJoint1Pos() + Math.PI / 2 - ArmConstants.ELEV_THETA) < 5 / 180. * Math.PI;
	}
}
