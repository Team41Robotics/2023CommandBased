package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;
import static java.lang.Math.PI;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmTo;
import frc.robot.commands.Balance;
import frc.robot.commands.GoTo;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Transform2d;

public class Autonomous {
	static ArmSubsystem arm = ArmSubsystem.getInstance();

	public static void initAutos() {
		// STUB
		// create("Basic Auton", () -> new SequentialCommandGroup(new GoTo(new Transform2d(-5.330, 2.755, -PI)))); //
		// wtf FIXME
		create("TAXI", () -> new GoTo(new Transform2d(1.02690 + 5, 2.73981, 0)));
		create("SCORE BALL", () -> new SequentialCommandGroup(new ArmTo("BALL TOP"), new RunIntake(.6, 1)));
		create(
				"Middle Auton",
				() -> new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 0.5, 2.73981, PI)),
						new ArmTo("BALL TOP"), // TODO intake
						new RunIntake(-.6, 1),
						new GoTo(new Transform2d(1.02690 + 5, 2.73981, PI)),
						new GoTo(new Transform2d(1.02690 + 3, 2.73981, PI)),
						new Balance()));
		create(
				"HP SIDE Auton",
				() -> new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 0.5, 4.41621, 0)),
						new ArmTo("BALL TOP"), // TODO intake
						new RunIntake(-.6, 1),
						new GoTo(new Transform2d(1.02690 + 5, 4.41621, 0))));
		create(
				"OTHER Auton",
				() -> new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 0.5, 1.06341, 0)),
						new ArmTo("BALL TOP"), // TODO intake
						new RunIntake(-.6, 1),
						new GoTo(new Transform2d(1.02690 + 5, 1.06341, 0))));
	}
}
