package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;
import static java.lang.Math.PI;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoTo;
import frc.robot.util.Transform2d;

public class Autonomous {
	public static void initAutos() {
		// STUB
		create("Basic Auton", () -> new SequentialCommandGroup(new GoTo(new Transform2d(-5.330, 2.755, -PI))));
	}
}
