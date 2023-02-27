package frc.robot.subsystems;

import static java.lang.Math.PI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
	CANSparkMax elev = new CANSparkMax(ArmConstants.ELEV_ID, MotorType.kBrushless);
	CANSparkMax joint1 = new CANSparkMax(ArmConstants.JOINT1_ID, MotorType.kBrushless);
	CANSparkMax joint2 = new CANSparkMax(ArmConstants.JOINT2_ID, MotorType.kBrushless);

	public ArmSubsystem() {
		zero();
	}

	public void zero() {
		elev.getEncoder().setPosition(ArmConstants.ELEV_PACKAGED_POSITION / 2 / PI / ArmConstants.ELEV_METERS_PER_RAD);
		joint1.getEncoder().setPosition(ArmConstants.JOINT1_PACKAGED_POSITION / 2 / PI / ArmConstants.JOINT1_RATIO);
		joint2.getEncoder().setPosition(ArmConstants.JOINT2_PACKAGED_POSITION / 2 / PI / ArmConstants.JOINT2_RATIO);
	}
	// TODO dynamic zeroing of encoder based on limit switches on elevator

	public double getElevPos() {
		return elev.getEncoder().getPosition() * 2 * PI * ArmConstants.ELEV_METERS_PER_RAD;
	}

	public double getJoint1Pos() {
		return joint1.getEncoder().getPosition() * 2 * PI / ArmConstants.JOINT1_RATIO;
	}

	public double getJoint2Pos() {
		return joint2.getEncoder().getPosition() * 2 * PI / ArmConstants.JOINT2_RATIO;
	}
}
