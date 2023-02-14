package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.HDriveSubsystem;

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
		double vf = -Util.deadZone(leftjs.getY()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double vs = -Util.deadZone(leftjs.getX()) * OperatorConstants.FWD_DRIVE_VELOCITY;
		double w = -Util.deadZone(rightjs.getX()) * OperatorConstants.TURN_VELOCITY;
		drive.drive(vf, 0*vs, w / 2);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
