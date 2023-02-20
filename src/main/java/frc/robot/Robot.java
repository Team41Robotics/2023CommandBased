package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.Balance;
import frc.robot.commands.Drive;
import frc.robot.commands.FODdrive;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	public static Joystick leftjs = new Joystick(0);
	public static Joystick rightjs = new Joystick(1);
	public static IMU imu = new IMU();
	HDriveSubsystem hdrive = HDriveSubsystem.getInstance();
	public boolean FOD;
	OdomSubsystem odom = OdomSubsystem.getInstance();
	PhotonVisionSubsystem pv = PhotonVisionSubsystem.getInstance();

	@Override
	public void robotInit() {
		hdrive.dttab.addBoolean("FOD", () -> FOD);
		configureButtons();
		hdrive.setDefaultCommand(new ConditionalCommand(new FODdrive(), new Drive(), () -> FOD));

		AutonomousRoutine.initShuffleboard();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance()
				.schedule(new SequentialCommandGroup(
						new GoTo(new Transform2d(1.02690 + 3, 2.73981, Math.PI)),
						new GoTo(new Transform2d(1.02690 + 2.4, 2.73981, Math.PI))));

	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		imu.zeroYaw();
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
	}

	public void configureButtons() {
		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		new JoystickButton(leftjs, 1).onTrue(new Balance().until(() -> rightjs.getRawButton(2)));
	}
}
