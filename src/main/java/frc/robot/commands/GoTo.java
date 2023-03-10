package frc.robot.commands;

import static frc.robot.Constants.GoToConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.util.Transform2d;
import frc.robot.util.Util;

public class GoTo extends CommandBase { // TODO: trajectory & make this more aggressive with traj
	HDriveSubsystem drive = HDriveSubsystem.getInstance();
	OdomSubsystem odom = OdomSubsystem.getInstance();

	Transform2d target;

	public GoTo(Transform2d target) {
		this(target, true);
	}

	public GoTo(Transform2d target, boolean transformIfRed) {
		if (transformIfRed && DriverStation.getAlliance() == Alliance.Red)
			target = Util.flipTransformAcrossField(target);
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
		if (abs(xPID.getPositionError()) < 0.1) xPID.setI(0.5);
		else xPID.setI(0);
		if (abs(yPID.getPositionError()) < 0.1) yPID.setI(0.5);
		else yPID.setI(0);
		if (abs(wPID.getPositionError()) < 10 * PI / 180) wPID.setI(1);
		else wPID.setI(0);

		double vx = xPID.calculate(odom.now().x, target.x);
		double vy = yPID.calculate(odom.now().y, target.y);
		double w = wPID.calculate(odom.now().theta, target.theta);

		double robot_angle = odom.now().theta;
		double vf = cos(robot_angle) * vx + sin(robot_angle) * vy;
		double vs = -sin(robot_angle) * vx + cos(robot_angle) * vy;
		drive.drive(vf, vs, w);
	}

	@Override
	public void end(boolean interrupted) {
		drive.drive(0, 0, 0);
	}

	public boolean isFinished() {
		return abs(odom.now().x - target.x) <= GOTO_XY_THRESHOLD
				// && abs(odom.now().y - target.y) <= GOTO_XY_THRESHOLD
				&& abs(Util.normRot(odom.now().theta - target.theta)) <= GOTO_TURN_THRESHOLD
				&& drive.getLeftVel() < GOTO_VEL_THRES
				&& drive.getRightVel() < GOTO_VEL_THRES;
		// && drive.getMidVel() < GOTO_VEL_THRES;
	}
}
