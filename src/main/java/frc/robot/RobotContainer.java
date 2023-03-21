package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.OdomSubsystem;
import frc.robot.subsystems.Operator;
import frc.robot.subsystems.PhotonVisionSubsystem;

public final class RobotContainer {
	public static Robot robot;

	public static Joystick leftjs = new Joystick(0);
	public static Joystick rightjs = new Joystick(1);
	public static Joystick DS = new Joystick(2);
	public static IMU imu = new IMU();

	public static HDriveSubsystem hdrive = new HDriveSubsystem();
	public static OdomSubsystem odom = new OdomSubsystem();
	public static PhotonVisionSubsystem pv = new PhotonVisionSubsystem();
	public static ArmSubsystem arm = new ArmSubsystem();
	public static LEDSubsystem leds = new LEDSubsystem();
	public static IntakeSubsystem intake = new IntakeSubsystem();
	public static Operator operator = new Operator();

	public static void initSubsystems() {
		hdrive.init();
		odom.init();
		pv.init();
		arm.init();
		leds.init();
		intake.init();
		operator.init();
		hdrive.initShuffleboard();
		odom.initShuffleboard();
		arm.initShuffleboard();
	}
}
