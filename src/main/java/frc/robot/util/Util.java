package frc.robot.util;

import static java.lang.Math.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;

public class Util {
	public static double deadZone(double joystickAxis, double deadZone) {
		if (abs(joystickAxis) < deadZone) return 0;
		return signum(joystickAxis) * (abs(joystickAxis) - deadZone) / (1 - deadZone);
	}

	public static double deadZone(double joystickAxis) {
		return deadZone(joystickAxis, OperatorConstants.JOYSTICK_DEADZONE);
	}

	public static double curvedDeadZone(double joystickAxis, double deadZone) {
		double deaded = deadZone(joystickAxis, deadZone);
		return deaded * abs(deaded); // TODO maybe change curve later
	}

	public static double curvedDeadZone(double joystickAxis) {
		return curvedDeadZone(joystickAxis, OperatorConstants.JOYSTICK_DEADZONE);
	}

	public static double normRot(double rad) {
		rad %= 2 * PI;
		if (rad > PI) rad -= 2 * PI;
		if (rad < -PI) rad += 2 * PI;
		return rad;
	}

	// lerps between t1 and t2; 0=t1, 1=t2
	public static Transform2d lerp(Transform2d t1, Transform2d t2, double alpha) {
		double tx = (1 - alpha) * t1.x + alpha * t2.x;
		double ty = (1 - alpha) * t1.y + alpha * t2.y;
		double theta = t1.theta + alpha * normRot(t2.theta - t1.theta);

		return new Transform2d(tx, ty, theta);
	}

	public static Transform2d flipPoseAcrossField(Transform2d trans) {
		return new Transform2d(Constants.FIELD_LENGTH - trans.x, trans.y, PI - trans.theta);
	}

	public static Color dim(Color color, double dimFactor) {
		int newRed = (int) (MathUtil.clamp(color.red * dimFactor, 0, 200));
		int newGreen = (int) (MathUtil.clamp(color.green * dimFactor, 0, 200));
		int newBlue = (int) (MathUtil.clamp(color.blue * dimFactor, 0, 200));

		return new Color(newRed, newGreen, newBlue);
	}

	public static double talonToRad(TalonFX talon) {
		return talon.getSelectedSensorPosition() / 2048. * 2 * PI;
	}

	public static double talonToRadPerSecond(TalonFX talon) {
		return talon.getSelectedSensorVelocity() * 10 / 2048. * 2 * PI;
	}

	public static double neoToRad(CANSparkMax neo) {
		return neo.getEncoder().getPosition() * 2 * PI;
	}

	public static double neoToRadPerSecond(CANSparkMax neo) {
		return neo.getEncoder().getPosition() * 2 * PI / 60.;
	}
}
