package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.util.Util;

public class Drive extends CommandBase {
	Joystick leftjs, rightjs;
	HDriveSubsystem drive = HDriveSubsystem.getInstance();

	public Drive() {
		addRequirements(drive);
		leftjs = Robot.leftjs;
		rightjs = Robot.rightjs;
	}

	@Override
	public void execute() {
		double vf = -Util.curvedDeadZone(leftjs.getY()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double vs = -Util.curvedDeadZone(leftjs.getX()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double w = -Util.curvedDeadZone(rightjs.getX()) * OperatorConstants.TURN_VELOCITY;
		int d = (rightjs.getRawButton(2)? 1 : 2);
		drive.drive(vf / d, vs / d, w / d); // TODO maybe turbo mode
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
