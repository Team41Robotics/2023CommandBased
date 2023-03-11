package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmTo;
import frc.robot.commands.Balance;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Transform2d;

public class Autonomous {
	static ArmSubsystem arm = ArmSubsystem.getInstance();

	public static void initAutos() {
		// STUB
		// create("Basic Auton", () -> new SequentialCommandGroup(new GoTo(new Transform2d(-5.330, 2.755, -PI)))); //
		// wtf FIXME
		create(
				"Middle Auton",
				() -> new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 0.5, 2.73981, 0)),
						new ArmTo("BALL HIGH"), // TODO intake
						new GoTo(new Transform2d(1.02690 + 4.5, 2.73981, 0)),
						new GoTo(new Transform2d(1.02690 + 3, 2.73981, 0)),
						new Balance()));
	}
}
