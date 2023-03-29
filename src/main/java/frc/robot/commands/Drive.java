package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.MechanicalConstants.DrivetrainConstants;
import frc.robot.util.Util;

public class Drive extends CommandBase {
	public Drive() {
		addRequirements(hdrive);
	}

	@Override
	public void execute() {
		double vf = -Util.curvedDeadZone(leftjs.getY()) * DrivetrainConstants.FWD_CONSTRAINTS.maxVelocity;
		double vs = -Util.curvedDeadZone(leftjs.getX()) * DrivetrainConstants.MID_CONSTRAINTS.maxVelocity;
		double w = -Util.curvedDeadZone(rightjs.getX())
				* DrivetrainConstants.FWD_CONSTRAINTS.maxVelocity
				/ DrivetrainConstants.RADIUS;
		int d = (rightjs.getRawButton(2) ? 1 : 2);
		hdrive.drive(vf / d, vs / d, w / d);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
