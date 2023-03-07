package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Transform2d;
import frc.robot.autonomous.AutonomousRoutine.AutonomousProvider;
import frc.robot.commands.GoTo;

public class Autonomous {
	static AutonomousProvider basic = () -> new SequentialCommandGroup(
			new GoTo(new Transform2d(1.02690 + 3, 2.73981, Math.PI)),
			new GoTo(new Transform2d(1.02690 + 2.4, 2.73981, Math.PI)));

	public static void initAutos() {
		// STUB
		// create("Basic Auton", () -> new SequentialCommandGroup(new GoTo(new Transform2d(-5.330, 2.755, -Math.PI))));
		create("Basic Auton", basic);
	}
}
