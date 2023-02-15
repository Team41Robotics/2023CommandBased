package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class Balance extends CommandBase {
	HDriveSubsystem drive = HDriveSubsystem.getInstance();
	OdomSubsystem odom = OdomSubsystem.getInstance();

	PIDController bal_pid = new PIDController(1, 0, 0);

	public Balance() {
		addRequirements(drive);
	}

	public void execute() {
		drive.drive(bal_pid.calculate(Robot.imu.getPitch(), 0), 0, 0);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(Robot.imu.getPitch()) < 1.5;
		// trivial
	}
}
