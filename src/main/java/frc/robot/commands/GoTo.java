package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.GoToConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Transform2d;
import frc.robot.util.Util;

public class GoTo extends CommandBase {
	Transform2d target;
	double fvx, fvy;

	public GoTo(Transform2d target) {
		this(target, true);
	}

	public GoTo(Transform2d target, boolean transformIfRed) {
		this(target, transformIfRed, 0, 0);
	}

	public GoTo(Transform2d target, boolean transformIfRed, double fvx, double fvy) {
		addRequirements(hdrive);
		this.target = target;
		boolean shift = transformIfRed && DriverStation.getAlliance() == Alliance.Red;
		if (shift) this.target = Util.flipPoseAcrossField(this.target);
		wPID.enableContinuousInput(-PI, PI);

		this.fvx = shift ? -fvx : fvx;
		this.fvy = fvy;
	}

	final PIDController xPID = new PIDController(3, 0, 0.4);
	final PIDController yPID = new PIDController(3, 0, 0.4);
	final PIDController wPID = new PIDController(1.3, 0, 0);

	@Override
	public void initialize() {
		xPID.reset();
		yPID.reset();
		wPID.reset();
	}

	@Override
	public void execute() {
		if (abs(xPID.getPositionError()) < 0.2) xPID.setI(0.5);
		else xPID.setI(0);
		if (abs(yPID.getPositionError()) < 0.2) yPID.setI(0.5);
		else yPID.setI(0);
		if (abs(wPID.getPositionError()) < 20 * PI / 180) wPID.setI(1);
		else wPID.setI(0);

		double vx = xPID.calculate(odom.now().x, target.x);
		double vy = yPID.calculate(odom.now().y, target.y);
		double w = wPID.calculate(odom.now().theta, target.theta);

		double robot_angle = odom.now().theta;
		double vf = cos(robot_angle) * vx + sin(robot_angle) * vy;
		double vs = -sin(robot_angle) * vx + cos(robot_angle) * vy;

		double max = hdrive.renormalize(vx, vy, w, 2);
		vf /= max;
		vs /= max;
		w /= max;

		double fvf = cos(robot_angle) * fvx + sin(robot_angle) * fvy;
		double fvs = -sin(robot_angle) * fvx + cos(robot_angle) * fvy;

		hdrive.drive(vf + fvf, vs + fvs, w, 1); // TODO refactor into parameter
	}

	@Override
	public void end(boolean interrupted) {
		hdrive.drive(0, 0, 0);
	}

	public boolean isFinished() {
		return abs(odom.now().x - target.x) <= GOTO_XY_TOLERANCE
				&& abs(odom.now().y - target.y) <= GOTO_XY_TOLERANCE
				&& abs(Util.normRot(odom.now().theta - target.theta)) <= GOTO_TURN_TOLERANCE
				&& hdrive.getLeftVel() < GOTO_VEL_TOLERANCE
				&& hdrive.getRightVel() < GOTO_VEL_TOLERANCE
				&& hdrive.getMidVel() < GOTO_VEL_TOLERANCE;
	}
}
