package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.Balance;
import frc.robot.commands.Drive;
import frc.robot.commands.FODdrive;
import frc.robot.commands.GoTo;
import frc.robot.commands.MovArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ZeroArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.subsystems.Operator;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.util.Transform2d;

public class Robot extends TimedRobot {
	public static Joystick leftjs = new Joystick(0);
	public static Joystick rightjs = new Joystick(1);
	public static Joystick DS = new Joystick(2);
	public static IMU imu = new IMU();

	public HDriveSubsystem hdrive = HDriveSubsystem.getInstance();
	public boolean FOD;
	public OdomSubsystem odom = OdomSubsystem.getInstance();
	public PhotonVisionSubsystem pv = PhotonVisionSubsystem.getInstance();
	public ArmSubsystem arm = ArmSubsystem.getInstance();
	public Command autonomousCommand;
	public LEDSubsystem lights = LEDSubsystem.getInstance();
	public IntakeSubsystem intake = IntakeSubsystem.getInstance();
	public Operator operator = Operator.getInstance();

	private void schedule(Command cmd) {
		CommandScheduler.getInstance().schedule(cmd);
	}

	@Override
	public void robotInit() {
		configureButtons();
		hdrive.dttab.addBoolean("FOD", () -> FOD);
		/*
		hdrive.setDefaultCommand(new ConditionalCommand(
				new InstantCommand(() -> {
					if (arm.getCurrentCommand() == null) {
						arm.set(
								Util.deadZone(-new Joystick(3).getY()),
								Util.deadZone(-rightjs.getY()),
								Util.deadZone(rightjs.getY()) + Util.deadZone(leftjs.getY()));
					}
				}),
				new Drive(),
				() -> FOD)); */
		hdrive.setDefaultCommand(new ConditionalCommand(new FODdrive(), new Drive(), () -> FOD));
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
		arm.elev.getEncoder().setPosition(0);
		if (autonomousCommand != null) {
			SequentialCommandGroup cmd =
					new SequentialCommandGroup(new WaitCommand(delay), new ZeroArm(), autonomousCommand);
			// SequentialCommandGroup cmd = new SequentialCommandGroup(new WaitCommand(delay), autonomousCommand);
			schedule(cmd);
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		// arm.set(0.0, 0, 0);
		odom.start();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	public void configureButtons() {
		new POVButton(leftjs, 0).onTrue(new InstantCommand(operator::moveUp));
		new POVButton(leftjs, 90).onTrue(new InstantCommand(operator::moveRight));
		new POVButton(leftjs, 180).onTrue(new InstantCommand(operator::moveDown));
		new POVButton(leftjs, 270).onTrue(new InstantCommand(operator::moveLeft));
		new JoystickButton(DS, 1).onTrue(new InstantCommand(operator::setPiece));
		// new JoystickButton(leftjs, 3).onTrue(new InstantCommand(operator::queuePlacement));

		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		new JoystickButton(leftjs, 4).onTrue(new Balance().until(() -> rightjs.getRawButton(2)));
		new JoystickButton(rightjs, 1).onTrue(new RunIntake(.6));
		new JoystickButton(leftjs, 1).onTrue(new RunIntake(-.6));
		new JoystickButton(rightjs, 2).onTrue(new RunIntake(0));

		new JoystickButton(leftjs, 3)
				.onTrue(new ProxyCommand(() -> new GoTo(new Transform2d(
						1.02690 + 1,
						(operator.queuedValue != null ? operator.queuedValue.getY() : 0) * 0.5588 + 2.73981,
						Math.PI))));
		// new JoystickButton(DS, 1).onTrue(new ArmTo(new ArmPosition(.5, 0, 0)));
		// new JoystickButton(DS, 1).onTrue(new InstantCommand(()->arm.set(.2,0,0)));
		new JoystickButton(DS, 5).whileTrue(new MovArm(0, -0.1, 1));
		new JoystickButton(DS, 6).whileTrue(new MovArm(0, 0.1, 1));
		// new JoystickButton(new Joystick(3), 1)
		//			.onTrue(new InstantCommand(() -> System.out.println("new ArmPosition(" + arm.getElevPos() + ","
		//					+ arm.getJoint1Pos() + "," + arm.getJoint2Pos() + ")")));

		/*
		new POVButton(leftjs, 270).onTrue(new RunCommand(() -> lights.flash(leftSide, Color.kYellow), lights));
		new POVButton(leftjs, 0).onTrue(new RunCommand(() -> lights.flash(midSide, Color.kYellow), lights));
		new POVButton(leftjs, 90).onTrue(new RunCommand(() -> lights.flash(rightSide, Color.kYellow), lights));
		new POVButton(leftjs, 180).onTrue(new RunCommand(() -> lights.flash(null, Color.kYellow), lights));
		new POVButton(rightjs, 270).onTrue(new RunCommand(() -> lights.flash(leftSide, Color.kPurple), lights));
		new POVButton(rightjs, 0).onTrue(new RunCommand(() -> lights.flash(midSide, Color.kPurple), lights));
		new POVButton(rightjs, 90).onTrue(new RunCommand(() -> lights.flash(rightSide, Color.kPurple), lights));
		new POVButton(rightjs, 180).onTrue(new RunCommand(() -> lights.flash(null, Color.kYellow), lights));
		*/
	}
}
