package frc.robot.commands;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
		if (DriverStation.getAlliance() == Alliance.Red) robot_angle = PI - robot_angle;
		double vx = cos(robot_angle) * vf + sin(robot_angle) * vs;
		double vy = -sin(robot_angle) * vf + cos(robot_angle) * vs;
		int d = (rightjs.getRawButton(2)? 1 : 2);

		drive.drive(vx / d, vy / d, w / d);
	}

	@Override
	public boolean isFinished() {
		return true;
		// trivial
	}
}
