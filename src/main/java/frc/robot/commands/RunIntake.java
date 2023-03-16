package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
	IntakeSubsystem intake = IntakeSubsystem.getInstance();
	double speed;
	double startTime;
	double mintime, maxtime;

	public RunIntake(double d) {
		this(d, .5, 99);
	}

	public RunIntake(double d, double maxtime) {
		this(d, .5, maxtime);
	}

	public RunIntake(double d, double mintime, double maxtime) {
		addRequirements(intake);
		this.speed = d;
		this.mintime = mintime;
		this.maxtime = maxtime;
	}

	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
		intake.run(speed);
	}

	@Override
	public void end(boolean interrupted) {
		intake.run(0);
	}

	@Override
	public boolean isFinished() {
		return (Math.abs(intake.getSpeed()) <= Math.abs(speed) / 2 && Timer.getFPGATimestamp() - startTime > mintime)
				|| Timer.getFPGATimestamp() - startTime > maxtime;
	}
}
