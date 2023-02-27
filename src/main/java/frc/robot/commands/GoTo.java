package frc.robot.commands;

import static java.lang.Math.PI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Transform2d;
import frc.robot.Util;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class GoTo extends CommandBase { // TODO: trajectory & make this more aggressive with traj
	HDriveSubsystem drive = HDriveSubsystem.getInstance();
	OdomSubsystem odom = OdomSubsystem.getInstance();

	Transform2d target;

	public GoTo(Transform2d target) {
		this.target = target;
		addRequirements(drive);
		wPID.enableContinuousInput(-PI, PI);
	}

	PIDController xPID = new PIDController(2, 0, 0);
	PIDController yPID = new PIDController(2, 0, 0);
	PIDController wPID = new PIDController(1.3, 0, 0);

	@Override
	public void initialize() {
		xPID.reset();
		yPID.reset();
		wPID.reset();
	}

	@Override
	public void execute() {
		if (Math.abs(xPID.getPositionError()) < 0.1) xPID.setI(0.5);
		else xPID.setI(0);
		if (Math.abs(yPID.getPositionError()) < 0.1) yPID.setI(0.5);
		else yPID.setI(0);
		if (Math.abs(wPID.getPositionError()) < 10 * PI / 180) wPID.setI(1);
		else wPID.setI(0);

		double vx = xPID.calculate(odom.now().x, target.x);
		double vy = yPID.calculate(odom.now().y, target.y);
		double w = wPID.calculate(odom.now().theta, target.theta);

		double robot_angle = odom.now().theta;
		double vf = Math.cos(robot_angle) * vx + Math.sin(robot_angle) * vy;
		double vs = -Math.sin(robot_angle) * vx + Math.cos(robot_angle) * vy;
		drive.drive(vf, vs, w);
	}

	public boolean isFinished() {
		return Math.abs(odom.now().x - target.x) <= Constants.GOTO_XY_THRESHOLD
				// && Math.abs(odom.now().y - target.y) <= Constants.GOTO_XY_THRESHOLD
				&& Math.abs(Util.normRot(odom.now().theta - target.theta)) <= Constants.GOTO_TURN_THRESHOLD
				&& drive.getLeftVel() < Constants.GOTO_VEL_THRES
				&& drive.getRightVel() < Constants.GOTO_VEL_THRES;
		// && drive.getMidVel() < Constants.GOTO_VEL_THRES;
	}
}
