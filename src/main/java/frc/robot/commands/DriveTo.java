package frc.robot.commands;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HDriveSubsystem;

public class DriveTo extends CommandBase {
	double dist, vel;
	HDriveSubsystem hd = HDriveSubsystem.getInstance();

	public DriveTo(double dist, double vel) {
		this.dist = dist;
		this.vel = vel;
		addRequirements(hd);
	}

	double lse, rse;

	@Override
	public void initialize() {
		lse = hd.getLeftPos();
		rse = hd.getRightPos();
	}

	@Override
	public void execute() {
		hd.drive(vel, 0, 0);
	}

	@Override
	public void end(boolean interrupted) {
		hd.drive(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return abs(hd.getLeftPos() + hd.getRightPos() - lse - rse) > abs(dist);
	}
}
