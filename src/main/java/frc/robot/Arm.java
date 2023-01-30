package frc.robot;

import java.util.Math;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm {

    private static CANSparkMax motor1;
    private static CANSparkMax motor2;
    private static CANSparkMax motor3;

    private static final int  length1 = 10;
    private static final int length2 = 5;
    private static final int length3 = 100;

    private static boolean engaged;

    private static DigitalInput originLimit;
    private static DigitalInput maximumLimit;

    private static RelativeEncoder motor1Encoder;
    private static RelativeEncoder motor2Encoder;
    private static RelativeEncoder motor3Encoder;

    SparkMaxPIDController motor1PIDController;
    SparkMaxPIDController motor2PIDController;
    SparkMaxPIDController motor3PIDController;

    public void initArm(){

        motor1 = new CANSparkMax(Constants.NEO_ONE_DEVICE_ID, MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.NEO_TWO_DEVICE_ID, MotorType.kBrushless);
        motor3 = new CANSparkMax(Constants.NEO_THREE_DEVICE_ID, MotorType.kBrushless);
        
        motor1Encoder = motor1.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
        motor2Encoder = motor2.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
        motor3Encoder = motor3.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);  

        originLimit = new DigitalInput(0);
        maximumLimit = new DigitalInput(1);

        if(!originLimit.get()){
            motor1Encoder.reset();
            motor2Encoder.reset();
            motor3Encoder.reset();
        }

        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor3.restoreFactoryDefaults();

        motor1PIDController = motor1.getPIDController();
        motor2PIDController = motor2.getPIDController();
        motor3PIDController = motor3.getPIDController();

        setPID(motor1PIDController, Constants.kP, Constants.kI, Constants.kD);
        setPID(motor2PIDController, Constants.kP, Constants.kI, Constants.kD);
        setPID(motor3PIDController, Constants.kP, Constants.kI, Constants.kD);

        motor1Encoder.setConversionFactor();
        motor2Encoder.setConversionFactor();
        motor3Encoder.setConversionFactor();

        }


    }

    public void setPID(SparkMaxPIDController controller, double p, double i, double d){
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
    }

    // WIP

    public double calculateVelocity (RelativeEncoder encoder){
        return -1;
    }

}   
