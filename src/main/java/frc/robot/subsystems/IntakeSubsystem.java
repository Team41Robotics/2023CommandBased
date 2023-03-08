package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase{

    static IntakeSubsystem intakeSubsystemInstance;
    CANSparkMax motor = new CANSparkMax(13, MotorType.kBrushless);

    public void runMotor(double x){
        motor.set(x);
    }

    public static IntakeSubsystem getIntakeInstace(){
        if(intakeSubsystemInstance == null){
            intakeSubsystemInstance = new IntakeSubsystem();
        }

        return intakeSubsystemInstance;
    }

}

