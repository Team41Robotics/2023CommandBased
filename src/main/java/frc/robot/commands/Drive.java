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

		double vf = -Util.curvedDeadZone(controller.getLeftY()) * DrivetrainConstants.FWD_CONSTRAINTS.maxVelocity;
		double vs = -Util.curvedDeadZone(controller.getLeftX()) * DrivetrainConstants.MID_CONSTRAINTS.maxVelocity;
		double w = -Util.curvedDeadZone(controller.getRightX())
				* DrivetrainConstants.FWD_CONSTRAINTS.maxVelocity
				/ DrivetrainConstants.RADIUS;
		double  d = slider.getDouble(0.25);
		System.out.println("d" +d );
		hdrive.drive(vf * d, vs * d, w * 0.5 * d);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
