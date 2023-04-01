package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.isCone;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase {
	final double speed;
	double startTime;
	final double mintime;
	final double maxtime;
	boolean automatic_choice;
	double maxVel = 0;

	public RunIntake(double d, double maxtime, boolean automatic_choice) {
		this(d, .5, maxtime);
		this.automatic_choice = automatic_choice;
	}

	public RunIntake(double d, boolean automatic_choice) {
		this(d, .5, 99);
		this.automatic_choice = automatic_choice;
	}

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
	public void initialize() { // automatic_choice FALSE: -cone +cube intake
		maxVel = 0;
		startTime = Timer.getFPGATimestamp();
		if (automatic_choice && !DriverStation.isAutonomous()) {
			int y = operator.queuedValue == null ? 0 : (int) operator.queuedValue.getY();
			intake.run(speed * (isCone[y] ? -1 : 1));
		} else intake.run(speed);
	}

	@Override
	public void execute() {
		maxVel = max(maxVel, abs(intake.getPercentSpeed()));
	}

	@Override
	public void end(boolean interrupted) {
		intake.run(0);
	}

	@Override
	public boolean isFinished() {
		return (abs(intake.getPercentSpeed()) <= maxVel / 2 && Timer.getFPGATimestamp() - startTime > mintime)
				|| Timer.getFPGATimestamp() - startTime > maxtime;
	}
}
