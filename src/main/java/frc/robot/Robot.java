package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.ArmTo;
import frc.robot.commands.Drive;
import frc.robot.commands.MovArm;
import frc.robot.commands.ZeroArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.util.ArmPosition;
import frc.robot.util.Util;

// CUBE PICKUP new ArmPosition(0.010986630314459667,-0.8002465405557883,1.0712389349851006)
// CUBE TOP new ArmPosition(0.8336088172302807,-0.574033297598237,0.8692904995316303)
// CUBE MID new ArmPosition(1.0313644120603538,0.038079522366785044,1.2038776589856977)
// ALLL BOT new ArmPosition(0.11673258803767007,-0.8509096153222929,1.1722132654382311)
// CONE MID new ArmPosition(0.4133713847787246,0.18337043505106876,0.37465626394903323)
// CONE TOP new ArmPosition(1.0409775388934257,0.10421664453550543,0.7535395422707984)
// CONE PICKUP new ArmPosition(0.14145238118888118,0.09577348684179293,-0.66101056063069)
// CUBE HIGH new ArmPosition(0.9505611193435574,0.24985805528722246,-0.14142111064490803)

public class Robot extends TimedRobot {
	public static Joystick leftjs = new Joystick(0);
	public static Joystick rightjs = new Joystick(1);
	public static Joystick DS = new Joystick(2);
	public static IMU imu = new IMU();

	HDriveSubsystem hdrive = HDriveSubsystem.getInstance();
	public boolean FOD;
	OdomSubsystem odom = OdomSubsystem.getInstance();
	// PhotonVisionSubsystem pv = PhotonVisionSubsystem.getInstance();
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
				() -> FOD));
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
		// schedule(new ZeroArm()); // TODO add to all auton stuffs
		schedule(new ZeroArm().andThen(new ArmTo(new ArmPosition(.5, 0, 0)))); // TODO add to all auton stuffs
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

		// if (leftjs.getRawButton(1)) hdrive.drive(OperatorConstants.FWD_DRIVE_VELOCITY / 20, 0, 0);
		// else if (rightjs.getRawButton(1)) hdrive.drive(-OperatorConstants.FWD_DRIVE_VELOCITY / 20, 0, 0);
		// else hdrive.drive(0, 0, 0);
		// if (leftjs.getRawButton(1)) schedule(new GoTo(new Transform2d(1, 0, PI)));
	}

	public void configureButtons() {
		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		// new JoystickButton(leftjs, 1).onTrue(new Balance().until(() -> rightjs.getRawButton(2)));
		new JoystickButton(rightjs, 1).onTrue(new RunCommand(() -> intake.run(.3), intake));
		new JoystickButton(leftjs, 1).onTrue(new RunCommand(() -> intake.run(-.3), intake));
		new JoystickButton(rightjs, 2).onTrue(new RunCommand(() -> intake.run(0), intake));
		// new JoystickButton(DS, 1).onTrue(new ArmTo(new ArmPosition(.5, 0, 0)));
		// new JoystickButton(DS, 1).onTrue(new InstantCommand(()->arm.set(.2,0,0)));
		new JoystickButton(DS, 1).whileTrue(new MovArm(0, -0.1, 1));
		new JoystickButton(new Joystick(3), 1)
				.onTrue(new InstantCommand(() -> System.out.println("new ArmPosition(" + arm.getElevPos() + ","
						+ arm.getJoint1Pos() + "," + arm.getJoint2Pos() + ")")));
	}
}
