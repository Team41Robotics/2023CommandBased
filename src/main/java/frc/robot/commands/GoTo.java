package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Transform2d;
import frc.robot.Util;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class GoTo extends CommandBase {
	HDriveSubsystem drive = HDriveSubsystem.getInstance();
	static OdomSubsystem odom = OdomSubsystem.getInstance();

	static Transform2d target;

	public GoTo(Transform2d target) {
		this.target = target;
		addRequirements(drive);
		wPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	static PIDController xPID = new PIDController(1.5, 0, 0);
	static PIDController yPID = new PIDController(1.5, 0, 0);
	static PIDController wPID = new PIDController(1.3, 0, 0);

	static {
		ShuffleboardTab gototab = Shuffleboard.getTab("GoTo");
		gototab.add("x", xPID);
		gototab.add("y", yPID);
		gototab.add("w", wPID);

                gototab.addNumber("x error", () -> odom.now().x - target.x);
                gototab.addNumber("y error", () -> odom.now().y - target.y);
                gototab.addNumber("theta error", () -> Util.normRot(odom.now().theta - target.theta)/Math.PI*180);
	}

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
		if (Math.abs(wPID.getPositionError()) < 10 * Math.PI / 180) wPID.setI(1);
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
				&& Math.abs(odom.now().y - target.y) <= Constants.GOTO_XY_THRESHOLD
				&& Math.abs(Util.normRot(odom.now().theta - target.theta)) <= Constants.GOTO_TURN_THRESHOLD;
	}
}
