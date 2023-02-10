package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.FODdrive;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.OdomSubsystem;

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	public static Joystick leftjs = new Joystick(0);
	public static Joystick rightjs = new Joystick(1);
	public static IMU imu = new IMU();
	HDriveSubsystem hdrive = HDriveSubsystem.getInstance();
	public boolean FOD;
	OdomSubsystem odom = OdomSubsystem.getInstance();
	// PhotonVisionSubsystem pv = PhotonVisionSubsystem.getInstance();

	@Override
	public void robotInit() {
		hdrive.dttab.addBoolean("FOD", () -> FOD);
		configureButtons();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		HDriveSubsystem.getInstance().setDefaultCommand(new ConditionalCommand(new FODdrive(), new Drive(), () -> FOD));
	}

	@Override
	public void autonomousInit() {}

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
	public void teleopPeriodic() {}

	public void configureButtons() {
		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		new JoystickButton(leftjs, 1)
				.onTrue(new GoTo(new Transform2d(14.513, 1.071 - 0.559, 0))
						// new Transform2d(2, 3, 0 * Math.PI / 2)) // CHANGE WITH COORD SYSTEM
						.until(() -> rightjs.getRawButtonPressed(2)));
		// OUTDATED
		// /*
		new JoystickButton(rightjs, 1)
				.onTrue(new SequentialCommandGroup(
								new GoTo(new Transform2d(0.5, 0.5, Math.PI)),
								new GoTo(new Transform2d(-0.5, 0.5, Math.PI / 2)),
								new GoTo(new Transform2d(0, 0, 0)))
						.repeatedly()
						.until(() -> rightjs.getRawButtonPressed(2)));
		// */
	}
}
