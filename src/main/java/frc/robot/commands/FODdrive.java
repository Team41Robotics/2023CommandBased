package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.MechanicalConstants.DrivetrainConstants;
import frc.robot.util.Util;

public class FODdrive extends CommandBase {
	public FODdrive() {
		addRequirements(hdrive);
	}

	public void execute() {
		double vf = -Util.curvedDeadZone(controller.getLeftY()) * DrivetrainConstants.FWD_CONSTRAINTS.maxVelocity;
		double vs = -Util.curvedDeadZone(controller.getLeftX()) * DrivetrainConstants.MID_CONSTRAINTS.maxVelocity;
		double w = -Util.curvedDeadZone(controller.getRightX(), .5)
				* DrivetrainConstants.FWD_CONSTRAINTS.maxVelocity
				/ DrivetrainConstants.RADIUS;

		double robot_angle = odom.now().theta;
		if (DriverStation.getAlliance() == Alliance.Red) robot_angle = PI + robot_angle;
		double vx = cos(robot_angle) * vf + sin(robot_angle) * vs;
		double vy = -sin(robot_angle) * vf + cos(robot_angle) * vs;
		int d = 4;

		hdrive.drive(vx / d, vy / d, w / 2 / d);
	}

	@Override
	public boolean isFinished() {
		return true;
		// trivial
	}
}
