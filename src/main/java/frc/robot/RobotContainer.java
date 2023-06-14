package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.HDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Operator;
import java.util.Map;

public final class RobotContainer {
	public static Robot robot;

	public static final CommandXboxController controller = new CommandXboxController(0);
	// public static final Joystick leftjs = new Joystick(0);
	// public static final Joystick rightjs = new Joystick(1);
	public static final Joystick DS = new Joystick(2);
	public static final IMU imu = new IMU();

	public static final HDrive hdrive = new HDrive();
	public static final Odometry odom = new Odometry();
	// public static final Vision pv = new Vision();
	public static final Arm arm = new Arm();
	public static final LEDs leds = new LEDs();
	public static final Intake intake = new Intake();
	public static final Operator operator = new Operator();

	public static final GenericEntry slider = Shuffleboard.getTab("Configs")
			.add("Max Speed", 1)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
			.getEntry();

	public static void initSubsystems() {
		slider.setDouble(0.25);
		hdrive.init();
		odom.init();
		// pv.init();
		arm.init();
		leds.init();
		intake.init();
		operator.init();
		hdrive.initShuffleboard();
		odom.initShuffleboard();
		arm.initShuffleboard();
		imu.initShuffleboard();
	}
}
