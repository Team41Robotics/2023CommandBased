package frc.robot;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Constants.ArmPos.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
import frc.robot.constants.MechanicalConstants.ArmConstants;
import frc.robot.util.Transform2d;

public class Robot extends TimedRobot {
	public boolean FOD;
	public Command autonomousCommand;

	public Robot() {
		super();
		try {
			PIDController.class.getDeclaredField("m_totalError").setAccessible(true);
		} catch (NoSuchFieldException e) {
			e.printStackTrace();
		} catch (SecurityException e) {
			e.printStackTrace();
		}
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

	@Override
	public void autonomousInit() {
		odom.start();

		autonomousCommand = AutonomousRoutine.AUTO_CHOOSER.getSelected().construct();
		double delay = AutonomousRoutine.AUTO_DELAY_CHOOSER.getSelected();
		if (autonomousCommand != null) {
			SequentialCommandGroup cmd = new SequentialCommandGroup(new WaitCommand(delay), autonomousCommand);
			// schedule(cmd);
		}
		schedule(new SequentialCommandGroup(
						new InstantCommand(() -> arm.jt2.set(.3), arm),
						new WaitUntilCommand(() -> !arm.joint2_limit.get()),
						new InstantCommand(() -> arm.jt2.set(0)),
						new InstantCommand(
								() -> arm.jt2.getEncoder().setPosition(2.5362667 * ArmConstants.JOINT2_RATIO / 2 / PI)),
						new ParallelCommandGroup(new SequentialCommandGroup(
								new InstantCommand(() -> arm.jt2.set(-.5)),
								new WaitUntilCommand(() -> arm.getJoint2Pos() < 1.2),
								new InstantCommand(() -> arm.jt2.set(0)))),
						new SequentialCommandGroup(
								new InstantCommand(() -> arm.jt1.set(.5)),
								new WaitUntilCommand(() ->
										arm.jt1.getEncoder().getVelocity() / 60 * 2 * PI / ArmConstants.JOINT1_RATIO
												> 0.1),
								new WaitUntilCommand(() ->
										arm.jt1.getEncoder().getVelocity() / 60 * 2 * PI / ArmConstants.JOINT1_RATIO
												< 0.1),
								new InstantCommand(() -> arm.jt1
										.getEncoder()
										.setPosition(16.15 / 180. * PI / 2 / PI * ArmConstants.JOINT1_RATIO)),
								new InstantCommand(() -> arm.jt1.set(0))))
				.asProxy()
				.andThen(new ArmTo(CONE_MID).asProxy()));
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		odom.start();
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
		int x = (operator.queuedValue != null ? operator.queuedValue.x : 0);
		if (x == 0) return ALL_BOT;
		return ArmPos.values()[x + (isCone[x] ? 3 : 0)];
	}

	public void configureButtons() { // TODO remap
		new JoystickButton(DS, 1).onTrue(new InstantCommand(operator::setPiece));

		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		new JoystickButton(leftjs, 4).onTrue(new Balance().until(() -> rightjs.getRawButton(2)));
		new JoystickButton(rightjs, 1).onTrue(new RunIntake(.6, true));
		new JoystickButton(leftjs, 1).onTrue(new RunIntake(-.6, true));
		new JoystickButton(rightjs, 2).onTrue(new RunIntake(0));

		new JoystickButton(leftjs, 3)
				.onTrue(new ProxyCommand(() -> new GoTo(new Transform2d(
								1.02690 + 1.5,
								(operator.queuedValue != null ? operator.queuedValue.getY() - 1 : -1) * -0.5588
										+ 4.41621,
								Math.PI)))
						.andThen(new ArmTo(getCurrentArm()).asProxy())
						.andThen(new RunIntake(-.6, true))
						.until(() -> rightjs.getRawButton(3)));
		new JoystickButton(DS, 5).whileTrue(new MovArm(0, -0.1, 1));
		new JoystickButton(DS, 6).whileTrue(new MovArm(0, 0.1, 1));
	}
}
