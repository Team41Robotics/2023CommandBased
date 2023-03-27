package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.util.Util;

public class FODdrive extends CommandBase {
	public FODdrive() {
		addRequirements(hdrive);
	}

	public void execute() {
		double vf = -Util.deadZone(leftjs.getY()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double vs = -Util.deadZone(leftjs.getX()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double w = -Util.deadZone(rightjs.getX()) * OperatorConstants.TURN_VELOCITY;

		double robot_angle = odom.now().theta;
		if (DriverStation.getAlliance() == Alliance.Red) robot_angle = PI - robot_angle;
		double vx = cos(robot_angle) * vf + sin(robot_angle) * vs;
		double vy = -sin(robot_angle) * vf + cos(robot_angle) * vs;
		int d = (rightjs.getRawButton(2) ? 1 : 2);

		hdrive.drive(vx / d, vy / d, w / d);
	}

	@Override
	public boolean isFinished() {
		return true;
		// trivial
	}
}
