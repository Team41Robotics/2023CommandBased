package frc.robot.commands;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.util.Util;

public class FODdrive extends CommandBase {
	Joystick leftjs, rightjs;
	HDriveSubsystem drive = HDriveSubsystem.getInstance();
	OdomSubsystem odom = OdomSubsystem.getInstance();

	public FODdrive() {
		addRequirements(drive);
		leftjs = Robot.leftjs;
		rightjs = Robot.rightjs;
	}

	public void execute() {
		double vf = -Util.deadZone(leftjs.getY()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double vs = -Util.deadZone(leftjs.getX()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double w = -Util.deadZone(rightjs.getX()) * OperatorConstants.TURN_VELOCITY;

		double robot_angle = odom.now().theta;
		double vx = cos(robot_angle) * vf + sin(robot_angle) * vs;
		double vy = -sin(robot_angle) * vf + cos(robot_angle) * vs;
		drive.drive(vx / 4, vy / 4, w / 2); // FIXME
	}

	@Override
	public void end(boolean interrupted) {
		drive.drive(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return true;
		// trivial
	}
}
