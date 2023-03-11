package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
	IntakeSubsystem intake = IntakeSubsystem.getInstance();
	double speed;
	double startTime;

	public RunIntake(double d) {
		addRequirements(intake);
		this.speed = d;
	}

	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
                System.out.println("EXECUTE");
		intake.run(speed);
	}

	@Override
	public boolean isFinished() {
		return (Math.abs(intake.getSpeed()) <= Math.abs(speed) / 2 && Timer.getFPGATimestamp() - startTime > 0.5);
	}
}
