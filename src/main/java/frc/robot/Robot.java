package frc.robot;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Constants.ArmPos.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.ArmTo;
import frc.robot.commands.Balance;
import frc.robot.commands.Drive;
import frc.robot.commands.FODdrive;
import frc.robot.commands.GoTo;
import frc.robot.commands.HoldArm;
import frc.robot.commands.MovArm;
import frc.robot.commands.RunIntake;
import frc.robot.util.Transform2d;

public class Robot extends TimedRobot {
	public boolean FOD;
	public Command autonomousCommand;

	public Robot() {
		super();
	}

	public void schedule(Command cmd) {
		CommandScheduler.getInstance().schedule(cmd);
	}

	@Override
	public void robotInit() {
		robot = this;
		initSubsystems();
		configureButtons();

		Shuffleboard.getTab("Drivetrain").addBoolean("FOD", () -> FOD);
		hdrive.setDefaultCommand(new ConditionalCommand(new FODdrive(), new Drive(), () -> FOD));
		arm.setDefaultCommand(new HoldArm());
		AutonomousRoutine.initShuffleboard();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	boolean has_been_enabled = false;

	public void enabledInit() {
		if (has_been_enabled) return;
		has_been_enabled = true;
		odom.start();
	}

	@Override
	public void autonomousInit() {
		enabledInit();

		autonomousCommand = AutonomousRoutine.AUTO_CHOOSER.getSelected().construct();
		double delay = AutonomousRoutine.AUTO_DELAY_CHOOSER.getSelected();
		if (autonomousCommand != null) {
			SequentialCommandGroup cmd = new SequentialCommandGroup(new WaitCommand(delay), autonomousCommand);
			schedule(cmd);
		}
		// schedule(new SequentialCommandGroup(
		// new ZeroArm().asProxy().andThen(new ArmTo(CONE_MID).asProxy()).andThen(new RunIntake(.6, 1))));
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		enabledInit();

		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		// arm.set(.5,0,0,0,0,0,0,0,0);
		arm.hold();
	}

	public ArmPos getCurrentArm() {
                System.out.println(operator.queuedValue);
		int x = (operator.queuedValue != null ? operator.queuedValue.x : 0);
		int y = (operator.queuedValue != null ? operator.queuedValue.y : 0);
                System.out.println(x);
		if (x == 2) return ALL_BOT;
                System.out.println(ArmPos.values()[x + (isCone[x] ? 0 : 3)].name());
		return ArmPos.values()[x + (isCone[y] ? 3 : 0)];
	}

	public void configureButtons() { // TODO remap
		//new JoystickButton(DS, 1).onTrue(new InstantCommand(operator::setPiece));

		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		new JoystickButton(leftjs, 4).onTrue(new Balance().until(() -> rightjs.getRawButton(2)));
		new JoystickButton(rightjs, 1).onTrue(new RunIntake(.6, true));
		new JoystickButton(leftjs, 1).onTrue(new RunIntake(-.6, true));
		new JoystickButton(rightjs, 2).onTrue(new RunIntake(0));

		new JoystickButton(leftjs, 3)
				.onTrue(new ProxyCommand(() -> new GoTo(new Transform2d(
								1.02690 + 1,
								(operator.queuedValue != null ? operator.queuedValue.getY() - 1 : -1) * -0.5588
										+ 4.41621,
								Math.PI)))
						.andThen(new ProxyCommand(() -> new ArmTo(getCurrentArm())))
						.andThen(new RunIntake(-.6, 1, true))
                                                .andThen(new ArmTo(BALL_SLIDE).asProxy())
                                                 
						.until(() -> rightjs.getRawButton(3)));
		//new JoystickButton(DS, 5).whileTrue(new MovArm(0, -0.1, 1));
		//new JoystickButton(DS, 6).whileTrue(new MovArm(0, 0.1, 1));
	}
}
