package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.PI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase {
	PIDController bal_pid = new PIDController(1.2, 0, 0);
	double last_time_valid = Timer.getFPGATimestamp();

	public Balance() {
		addRequirements(hdrive);
	}

	public void execute() {
		if (Math.abs(bal_pid.getPositionError()) < 0.1) bal_pid.setI(0.1);
		else bal_pid.setI(0);
		hdrive.drive(bal_pid.calculate(imu.getPitch(), 0), 0, 0);

		if (Math.abs(imu.getPitch()) > 1.5 / 180 * PI) last_time_valid = Timer.getFPGATimestamp();
	}

	@Override
	public void end(boolean interrupted) {
		hdrive.drive(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - last_time_valid > 2;
		// trivial
	}
}
