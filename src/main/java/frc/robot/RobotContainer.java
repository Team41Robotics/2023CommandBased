package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.HDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Operator;
import frc.robot.subsystems.Vision;

public final class RobotContainer {
	public static Robot robot;

	public static Joystick leftjs = new Joystick(0);
	public static Joystick rightjs = new Joystick(1);
	public static Joystick DS = new Joystick(2);
	public static IMU imu = new IMU();

	public static HDrive hdrive = new HDrive();
	public static Odometry odom = new Odometry();
	public static Vision pv = new Vision();
	public static Arm arm = new Arm();
	public static LEDs leds = new LEDs();
	public static Intake intake = new Intake();
	public static Operator operator = new Operator();

	public static void initSubsystems() {
		hdrive.init();
		odom.init();
		pv.init();
		arm.init();
		leds.init();
		intake.init();
		operator.init();
		// hdrive.initShuffleboard();
		// odom.initShuffleboard();
		// arm.initShuffleboard();
		// imu.initShuffleboard();
	}
}
