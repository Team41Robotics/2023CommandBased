package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.util.Util;

public class Drive extends CommandBase {
	public Drive() {
		addRequirements(hdrive);
	}

	@Override
	public void execute() {
		double vf = -Util.curvedDeadZone(leftjs.getY()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double vs = -Util.curvedDeadZone(leftjs.getX()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double w = -Util.curvedDeadZone(rightjs.getX()) * OperatorConstants.TURN_VELOCITY;
		int d = (rightjs.getRawButton(2) ? 1 : 2);
		hdrive.drive(vf / d, vs / d, w / d);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
