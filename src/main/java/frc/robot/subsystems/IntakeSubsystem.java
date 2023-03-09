package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
	static IntakeSubsystem intake;
	CANSparkMax motor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);

	public void run(double x) {
		System.out.println(x);
		motor.set(x);
	}

	public static IntakeSubsystem getInstance() {
		if (intake == null) {
			intake = new IntakeSubsystem();
		}
		return intake;
	}
}
