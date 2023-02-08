package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Transform2d;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class GoTo extends CommandBase {
	Joystick leftjs, rightjs;
	HDriveSubsystem drive = HDriveSubsystem.getInstance();
	OdomSubsystem odom = OdomSubsystem.getInstance();

	Transform2d target;

	public GoTo(Transform2d target) {
		this.target = target;
		addRequirements(drive);
		leftjs = Robot.leftjs;
		rightjs = Robot.rightjs;
		wPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	PIDController xPID = new PIDController(7, 0, 0);
	PIDController yPID = new PIDController(7, 0, 0);
	PIDController wPID = new PIDController(5, 0, 1);

	@Override
	public void initialize() {
		xPID.reset();
		yPID.reset();
		wPID.reset();
	}

	@Override
	public void execute() {
		if (Math.abs(xPID.getPositionError()) < 0.1) xPID.setI(0.5);
		if (Math.abs(yPID.getPositionError()) < 0.1) yPID.setI(0.5);
		if (Math.abs(wPID.getPositionError()) < 10 * Math.PI / 180) wPID.setI(0.5);

		double vx = xPID.calculate(odom.now().x, target.x);
		double vy = yPID.calculate(odom.now().y, target.y);
		double w = wPID.calculate(odom.now().theta, target.theta);

		double robot_angle = odom.now().theta;
		double vf = Math.cos(robot_angle) * vx + Math.sin(robot_angle) * vy;
		double vs = -Math.sin(robot_angle) * vx + Math.cos(robot_angle) * vy;
		drive.drive(vf, vs, w);
	}

	public boolean isFinished() {
		return Math.abs(odom.now().x - target.x) <= 0.03
				&& Math.abs(odom.now().y - target.y) <= 0.03
				&& Math.abs(odom.now().theta - target.theta) <= 4 / 180. * Math.PI;
	}
}
