package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.ZeroArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.util.Util;

public class Robot extends TimedRobot {
	public static Joystick leftjs = new Joystick(0);
	public static Joystick rightjs = new Joystick(1);
	public static Joystick DS = new Joystick(2);
	public static IMU imu = new IMU();

	HDriveSubsystem hdrive = HDriveSubsystem.getInstance();
	public boolean FOD;
	OdomSubsystem odom = OdomSubsystem.getInstance();
	PhotonVisionSubsystem pv = PhotonVisionSubsystem.getInstance();
	ArmSubsystem arm = ArmSubsystem.getInstance();
	private Command autonomousCommand;
	LEDSubsytem lights = LEDSubsytem.getInstance();
	IntakeSubsystem intake = IntakeSubsystem.getInstance();

	private void schedule(Command cmd) {
		CommandScheduler.getInstance().schedule(cmd);
	}

	@Override
	public void robotInit() {
		configureButtons();
		lights.initLights();
		hdrive.dttab.addBoolean("FOD", () -> FOD);
		// hdrive.setDefaultCommand(new ConditionalCommand(new FODdrive(), new Drive(), () -> FOD));
		AutonomousRoutine.initShuffleboard();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = AutonomousRoutine.AUTO_CHOOSER.getSelected().construct();
		double delay = AutonomousRoutine.AUTO_DELAY_CHOOSER.getSelected();
		arm.elev.getEncoder().setPosition(0);
		arm.zero();
		if (autonomousCommand != null) {
			SequentialCommandGroup cmd = new SequentialCommandGroup(new WaitCommand(delay), autonomousCommand);
			schedule(cmd);
		}
		schedule(new ZeroArm()); // TODO add to all auton stuffs
		// schedule(new ZeroArm().andThen(new ArmTo(new ArmPosition(.5, 0, 0)))); // TODO add to all auton stuffs
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		arm.set(0.0, 0, 0);
		imu.zeroYaw(); // TODO move to auton
		odom.start();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		if (arm.getCurrentCommand() == null) {
			arm.set(
					Util.deadZone(-leftjs.getY()),
					Util.deadZone(-rightjs.getY()),
					Util.deadZone(rightjs.getY()) + Util.deadZone(-new Joystick(3).getY()));
		}
		// if (leftjs.getRawButton(1)) hdrive.drive(OperatorConstants.FWD_DRIVE_VELOCITY / 20, 0, 0);
		// else if (rightjs.getRawButton(1)) hdrive.drive(-OperatorConstants.FWD_DRIVE_VELOCITY / 20, 0, 0);
		// else hdrive.drive(0, 0, 0);
		// if (leftjs.getRawButton(1)) schedule(new GoTo(new Transform2d(1, 0, PI)));
	}

	public void configureButtons() {
		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		// new JoystickButton(leftjs, 1).onTrue(new Balance().until(() -> rightjs.getRawButton(2)));
		new JoystickButton(rightjs, 1).onTrue(new RunCommand(() -> intake.run(.6), intake));
		new JoystickButton(leftjs, 1).onTrue(new RunCommand(() -> intake.run(-.6), intake));
		new JoystickButton(rightjs, 2).onTrue(new RunCommand(() -> intake.run(0), intake));
		// new JoystickButton(DS, 1).onTrue(new ArmTo(new ArmPosition(.5, 0, 0)));
		// new JoystickButton(DS, 1).onTrue(new InstantCommand(()->arm.set(.2,0,0)));
	}
}
