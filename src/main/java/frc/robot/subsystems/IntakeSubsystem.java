package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
	CANSparkMax motor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);

	public void init() {
		motor.restoreFactoryDefaults();
		motor.setIdleMode(IdleMode.kBrake);
	}

	public void run(double x) {
		motor.set(x);
	}

	public double getPercentSpeed() {
		return motor.getEncoder().getVelocity() / Constants.NEO_550_MAX_SPEED_RPM;
	}
}
