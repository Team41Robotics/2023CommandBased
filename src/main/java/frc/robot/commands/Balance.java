package frc.robot.commands;

import static java.lang.Math.PI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class Balance extends CommandBase {
	HDriveSubsystem drive = HDriveSubsystem.getInstance();
	OdomSubsystem odom = OdomSubsystem.getInstance();

	PIDController bal_pid = new PIDController(1.2, 0, 0);
	double last_time_valid = Timer.getFPGATimestamp();

	public Balance() {
		addRequirements(drive);
	}

	public void execute() {
		if (Math.abs(bal_pid.getPositionError()) < 0.1) bal_pid.setI(0.1);
		else bal_pid.setI(0);
		drive.drive(bal_pid.calculate(Robot.imu.getPitch(), 0), 0, 0);

		if (Math.abs(Robot.imu.getPitch()) > 1.5 / 180 * PI) last_time_valid = Timer.getFPGATimestamp();
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - last_time_valid > 2;
		// trivial
	}
}
