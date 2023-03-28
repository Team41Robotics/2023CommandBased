package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Ports;

public class Intake extends SubsystemBase {
	final CANSparkMax motor = new CANSparkMax(Ports.CAN_INTAKE, MotorType.kBrushless);

	public void init() {
		motor.restoreFactoryDefaults();
		motor.setIdleMode(IdleMode.kBrake);
	}

	public void run(double x) {
		motor.set(x);
	}

	public double getPercentSpeed() {
		return motor.getEncoder().getVelocity() / 11710;
	}
}
