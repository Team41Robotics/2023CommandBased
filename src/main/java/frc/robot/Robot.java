package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousRoutine;
import frc.robot.commands.Balance;
import frc.robot.commands.Drive;
import frc.robot.commands.FODdrive;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class Robot extends TimedRobot {
	public static Joystick leftjs = new Joystick(0);

	public static Joystick rightjs = new Joystick(1);
	public static IMU imu = new IMU();
	HDriveSubsystem hdrive = HDriveSubsystem.getInstance();
	public boolean FOD;
	OdomSubsystem odom = OdomSubsystem.getInstance();
	PhotonVisionSubsystem pv = PhotonVisionSubsystem.getInstance();
	LEDSubsytem lights = LEDSubsytem.getInstance();
	private Command autonomousCommand;

	private void schedule(Command cmd) {
		CommandScheduler.getInstance().schedule(cmd);
	}

        DigitalInput a = new DigitalInput(0);
        DigitalInput b = new DigitalInput(1);
        CANSparkMax leftElevator =       new CANSparkMax(9, MotorType.kBrushless);
        CANSparkMax rightElevator = new CANSparkMax(8, MotorType.kBrushless);
	@Override
	public void robotInit() {
                hdrive.dttab.addBoolean("L1", ()->a.get());
                hdrive.dttab.addBoolean("L2", ()->b.get());

                
		configureButtons();
		lights.initLights();
		hdrive.dttab.addBoolean("FOD", () -> FOD);
		hdrive.setDefaultCommand(new ConditionalCommand(new FODdrive(), new Drive(), () -> FOD));

		AutonomousRoutine.initShuffleboard();

                leftElevator.setIdleMode(IdleMode.kBrake);
                rightElevator.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void robotPeriodic() {
                System.out.println(leftElevator.getEncoder().getPosition());
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = AutonomousRoutine.AUTO_CHOOSER.getSelected().construct();
		double delay = AutonomousRoutine.AUTO_DELAY_CHOOSER.getSelected();

		if (autonomousCommand != null) {
			SequentialCommandGroup cmd = new SequentialCommandGroup(new WaitCommand(delay), autonomousCommand);
			schedule(cmd);
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
                leftElevator.getEncoder().setPosition(0);
		imu.zeroYaw(); // TODO move to auton
		odom.start();
                limitSwitch=true;
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}
        private boolean limitSwitch = true;
	@Override
	public void teleopPeriodic() {
                if(limitSwitch) {
                        rightElevator.set(0.5);
                        leftElevator.set(0.5);
                 limitSwitch=a.get() && b.get();  }else{
                        leftElevator.set(-0.1);
                        rightElevator.set(-0.1);
                        limitSwitch=leftElevator.getEncoder().getPosition()<=1;
                }
               
		// if (leftjs.getRawButton(1)) hdrive.drive(OperatorConstants.FWD_DRIVE_VELOCITY / 20, 0, 0);
		// else if (rightjs.getRawButton(1)) hdrive.drive(-OperatorConstants.FWD_DRIVE_VELOCITY / 20, 0, 0);
		// else hdrive.drive(0, 0, 0);
		// if (leftjs.getRawButton(1)) schedule(new GoTo(new Transform2d(1, 0, Math.PI)));
	}

	public void configureButtons() {
		new JoystickButton(rightjs, 3)
				.onTrue(new InstantCommand(
						() -> System.out.println(odom.now().x + " : " + odom.now().y + " : " + odom.now().theta)));
		new JoystickButton(leftjs, 2).onTrue(new InstantCommand(() -> FOD = !FOD));
		new JoystickButton(leftjs, 1).onTrue(new Balance().until(() -> rightjs.getRawButton(2)));
	}
}
