package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;
import static java.lang.Math.PI;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmTo;
import frc.robot.commands.Balance;
import frc.robot.commands.GoTo;
import frc.robot.commands.RunIntake;
import frc.robot.util.Transform2d;

public class Autonomous {
	public static void initAutos() {
		// TODO test all
		create("TAXI", () -> new GoTo(new Transform2d(1.02690 + 5, 2.73981, 0)));
		create("SCORE BALL", () -> new SequentialCommandGroup(new ArmTo("BALL TOP"), new RunIntake(.6, 1)));
		create(
				"Middle Auton",
				() -> new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 1, 2.73981, PI)), // TODO fix & tune length later
						new ArmTo("BALL TOP"),
						new RunIntake(-.6, 1),
						new GoTo(new Transform2d(1.02690 + 5, 2.73981, PI)),
						new GoTo(new Transform2d(1.02690 + 3, 2.73981, PI)),
						new Balance()));
		create(
				"HP SIDE Auton",
				() -> new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 0.5, 4.41621, PI)),
						new ArmTo("BALL TOP"),
						new RunIntake(-.6, 1),
						new GoTo(new Transform2d(1.02690 + 5, 4.41621, 0))));
		create(
				"OTHER Auton",
				() -> new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 0.5, 1.06341, PI)),
						new ArmTo("BALL TOP"),
						new RunIntake(-.6, 1),
						new GoTo(new Transform2d(1.02690 + 5, 1.06341, 0))));
	}
}
