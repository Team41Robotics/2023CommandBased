package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;
import static frc.robot.constants.Constants.ArmPos.*;
import static java.lang.Math.PI;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmTo;
import frc.robot.commands.Balance;
import frc.robot.commands.GoTo;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ZeroArm;
import frc.robot.constants.Constants.ArmPos;
import frc.robot.util.Transform2d;

public class Autonomous {
        // spotless:off
	static Command createScoreAuton(double y, ArmPos first, ArmPos second) {
		return new SequentialCommandGroup(
                        new ZeroArm().asProxy(),
                        new ParallelCommandGroup(
                                new ArmTo(first).asProxy(),
                                new GoTo(new Transform2d(1.02690 + 1, y, PI))
                        ),
                        new RunIntake(-.6, .5),
                        new ArmTo(second).asProxy());
	}

	public static void initAutos() {
		create(
                        "Taxi Balance Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(2.73981, BALL_TOP, CONE_SLIDE),
                                new GoTo(new Transform2d(1.02690 + 5, 2.73981, PI)),
                                new GoTo(new Transform2d(1.02690 + 3, 2.73981, PI)),
                                new Balance()
                        )
                );
		create(
                        "Taxi HP SIDE Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(4.41621, BALL_TOP, CONE_SLIDE),
                                new GoTo(new Transform2d(1.02690 + 5, 4.41621, 0))
                        )
                );
		create(
                        "Taxi OTHER Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(1.05341, BALL_TOP, CONE_SLIDE),
                                new GoTo(new Transform2d(1.02690 + 5, 1.06341, 0))
                        )
                );
		create(
                        "OTHER 22BALL Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(1.05341, BALL_TOP, BALL_PICKUP),

                                // there
                                new GoTo(new Transform2d(3.02690, 0.86341, -10 / 180. * PI), true, 4, 0),
                                new ParallelDeadlineGroup(
                                        new GoTo(new Transform2d(6.02690, 0.86341, 0)),
                                        new RunIntake(.6)
                                ),

                                // back
                                new GoTo(new Transform2d(3.02690, 0.86341, -10 / 180. * PI), true, -2, 0),
                                new GoTo(new Transform2d(2.02690, 1.06341, 0)),

                                // score
                                new ArmTo(BALL_TOP).asProxy(),
                                new RunIntake(-.6, .5),
                                new ArmTo(CONE_SLIDE).asProxy()
                        )
                );
	}
        // spotless:on
}
