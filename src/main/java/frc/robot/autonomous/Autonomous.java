package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousRoutine.create;
import static frc.robot.constants.Constants.ArmPos.*;
import static java.lang.Math.PI;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
                        new RunIntake(.6, .5),
                        new ParallelCommandGroup(
                                new GoTo(new Transform2d(1.02690 + 1.5, y, PI)),
                                new ZeroArm().asProxy()
                        ),
                        new ArmTo(first).asProxy(),
                        new GoTo(new Transform2d(1.02690 + .9, y, PI)),
                        new RunIntake(-.6, .5),
                        new ArmTo(second).asProxy()
                );
	}

	public static void initAutos() {
                // HP SIDE AUTONS
                create(
                        "HP SIDE Score Auton",
                        () -> createScoreAuton(4.41621, BALL_TOP, BALL_SLIDE)
                );
		create(
                        "Taxi HP SIDE Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(4.41621, BALL_TOP, BALL_SLIDE),
                                new GoTo(new Transform2d(1.02690 + 5, 4.41621, PI))
                        )
                );
                // CENTER AUTONS
                create(
                        "Center Score Auton",
                        () -> createScoreAuton(2.73981, BALL_TOP, BALL_SLIDE)
                );
		create(
                        "Balance Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(2.73981, BALL_TOP, BALL_SLIDE),
                                new GoTo(new Transform2d(1.02690 + 1.2, 2.73891, 0)),
                                new GoTo(new Transform2d(1.02690 + 3, 2.73981, 0)),
                                new Balance()
                        )
                );
		create(
                        "Taxi Balance Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(2.73981, BALL_TOP, BALL_SLIDE),
                                new GoTo(new Transform2d(1.02690 + 1.2, 2.73891, 0)),
                                new GoTo(new Transform2d(1.02690 + 5, 2.73981, 0)),
                                new GoTo(new Transform2d(1.02690 + 3, 2.73981, 0)),
                                new Balance()
                        )
                );
                // OTHER SIDE AUTONS
                create(
                        "OTHER Score Auton",
                        () -> createScoreAuton(1.05314, BALL_TOP, BALL_SLIDE)
                );
		create(
                        "Taxi OTHER Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(1.05341, BALL_TOP, BALL_SLIDE),
                                new GoTo(new Transform2d(1.02690 + 5, 1.06341, PI))
                        )
                );
		create(
                        "2BALL OTHER Auton",
                        () -> new SequentialCommandGroup(
                                createScoreAuton(1.05341, BALL_TOP, BALL_PICKUP),
                                new GoTo(new Transform2d(1.02690 + 1.2, 1.05341, 0)),
                                new ParallelRaceGroup(
                                        new GoTo(new Transform2d(1.02690 + 3, 1.05341, 0)),
                                        new RunIntake(.6)
                                ),
                                new GoTo(new Transform2d(1.02690 + 1.2, 1.05341, 0)),
                                new GoTo(new Transform2d(1.02690 + 1.2, 1.05341, PI)),
                                new ArmTo(BALL_TOP).asProxy(),
                                new GoTo(new Transform2d(1.02690 + .9, 1.05341, PI)),
                                new RunIntake(-.6, .5),
                                new ArmTo(BALL_SLIDE).asProxy()
                        )
                );
	}
        // spotless:on
}
