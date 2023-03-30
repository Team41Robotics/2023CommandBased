package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.MechanicalConstants.ArmConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.util.Util;

public class MovArm extends CommandBase {
	double st;
	final double vx;
	final double vy;
	final double t;
	double e, j1, j2;

	public MovArm(double vx, double vy, double t) {
		addRequirements(arm);

		this.vx = vx;
		this.vy = vy;
		this.t = t;
	}

	@Override
	public void initialize() {
		st = Timer.getFPGATimestamp();
		e = arm.getElevPos();
		j1 = arm.getJoint1Pos();
		j2 = arm.getJoint2Pos();
	}

	@Override
	public void execute() {
		double alpha = arm.getJoint1Pos();
		double theta = ELEV_THETA;
		double sec = 1 / cos(alpha - theta);
		double v = sec * (vx * cos(alpha) + vy * sin(alpha));
		double omega = sec / ARM_LEN * (-vx * sin(theta) + vy * cos(theta));

		double max = 1;
		max = max(max, abs(v) / ELEV_CONSTRAINTS.maxVelocity);
		max = max(max, abs(omega) / JOINT1_CONSTRAINTS.maxVelocity);
		max = max(max, abs(omega) / JOINT2_CONSTRAINTS.maxVelocity);

		v /= max;
		omega /= max;

		e += v * Constants.LOOP_TIME;
		j1 += omega * Constants.LOOP_TIME;
		j2 -= omega * Constants.LOOP_TIME;

		arm.set(e, j1, j2, v, omega, -omega);
	}

	@Override
	public boolean isFinished() {
		double alpha = arm.getJoint1Pos();
		double theta = ELEV_THETA;
		double sec = 1 / cos(alpha - theta);
		double v = sec * (vx * cos(alpha) + vy * sin(alpha));
		if (arm.isBotLimitSwitch() && v < 0) return true;
		return Timer.getFPGATimestamp() > st + t
				|| Util.normRot(arm.getJoint1Pos() + Math.PI / 2 - ELEV_THETA) < 5 / 180. * Math.PI;
	}
}
