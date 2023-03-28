package frc.robot;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.MechanicalConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.ArmTo;
import frc.robot.commands.Drive;
import frc.robot.commands.FODdrive;
import frc.robot.commands.HoldArm;

public class Robot extends TimedRobot {
	public boolean FOD;
	public Command autonomousCommand;

	public Robot() {
		super();
		try {
			PIDController.class.getDeclaredField("m_totalError").setAccessible(true);
		} catch (NoSuchFieldException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SecurityException e) {
			// TODO Auto-generated catch block
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
				new InstantCommand(() -> arm.jt2.set(.3)),
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
								arm.jt1.getEncoder().getVelocity() / 60 * 2 * PI / ArmConstants.JOINT1_RATIO > 0.1),
						new WaitUntilCommand(() ->
								arm.jt1.getEncoder().getVelocity() / 60 * 2 * PI / ArmConstants.JOINT1_RATIO < 0.1),
						new InstantCommand(() -> arm.jt1
								.getEncoder()
								.setPosition(16.15 / 180. * PI / 2 / PI * ArmConstants.JOINT1_RATIO)),
						new InstantCommand(() -> arm.jt1.set(0))),
				new ArmTo("BALL TOP"),
				new WaitCommand(2),
				new ArmTo("BALL MID"),
				new WaitCommand(2),
				new ArmTo("CONE MID"),
				new WaitCommand(2),
				new ArmTo("CONE TOP")));
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

	public void configureButtons() { // TODO remap
	}
}
