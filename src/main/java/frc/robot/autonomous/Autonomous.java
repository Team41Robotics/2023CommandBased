package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmTo;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.ArmSubsystem;

public class Autonomous {
	static ArmSubsystem arm = ArmSubsystem.getInstance();

	public static void initAutos() {
		create("SCORE BALL", () -> new SequentialCommandGroup(new ArmTo("BALL TOP"), new RunIntake(.6, 1)));
	}
}
