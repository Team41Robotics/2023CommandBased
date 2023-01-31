
package frc.robot.subsystems;

import java.util.Math;
import java.util.HashMap; // <3

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;


public class ArmSubsystem extends Subsystem{

    private ArmSubsystem armSubsystem;
    
    private static CANSparkMax motor1;
    private static CANSparkMax motor2;
    private static CANSparkMax motor3;

    private static PIDController elevatorPID;
    private static PIDController armPID1;
    private static PIDController armPID2;


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

        /* 
        if(!originLimit.get()){
            motor1Encoder.reset();
            motor2Encoder.reset();
            motor3Encoder.reset();
        }
        */

        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor3.restoreFactoryDefaults();

        motor1PIDController = motor1.getPIDController();
        motor2PIDController = motor2.getPIDController();
        motor3PIDController = motor3.getPIDController();

        setPID(motor1PIDController, Constants.kP, Constants.kI, Constants.kD);
        setPID(motor2PIDController, Constants.kP, Constants.kI, Constants.kD);
        setPID(motor3PIDController, Constants.kP, Constants.kI, Constants.kD);

        /* 
        motor1Encoder.setConversionFactor();
        motor2Encoder.setConversionFactor();
        motor3Encoder.setConversionFactor();
        */

        elevatorPID = new PIDController(1,0,0);
        armPID1 = new PIDController(1,0,0);
        armPID2 = new PIDController(1,0,0);

        }

    }

    public void setPID(SparkMaxPIDController controller, double p, double i, double d){

        controller.setP(p);
        controller.setI(i);
        controller.setD(d);

    }

    // WIP

    @Override
    public void periodic(){
        motor1.set(elevatorPID.calculate(motor1Encoder.getDistance()), elevatorPID.getSetpoint());
    }

    public void targetSet(double h, double theta1, double theta2){

        double r = ArmConstants.JOINT_LENGTH;
        double rArmTheta = (ArmConstants.ARM_THETA * Math.PI)/180;
        
        double rTheta1 = (theta1*Math.PI())/180; // theta 1 in radians
        double rTheta2 = (theta2*Math.PI())/180; // theta 2 in radians

        double A = Math.tan(rArmTheta);
        double B = -1;
        double C = 0; // we can assume this because jason said we can

        double normAB = sqrt((A*A)+(B*B));
        
        double newH = C/normAB;
        double abV = (A*B)/normAB;

        if(r == h){
            elevatorPID.setPoint(h*abV);
        }else if(r > h){
            // No clue wtf to set these to
            elevatorPID.setPoint();
        }else{}

    }

    public static ArmSubsystem getInstance() {
        if(armSubsystem == null){
            this.armSubsystem = new ArmSubsystem();
        }

        return armSubsystem;

    } 

}   
