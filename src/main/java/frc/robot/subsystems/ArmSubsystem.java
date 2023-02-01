
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

    private static ArmSubsystem armSubsystem;
    
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
    public void periodic(){ // FIXME
        motor1.set(elevatorPID.calculate(motor1Encoder.getDistance()), elevatorPID.getSetpoint());
    }

    public double[][] getTarget(double x, double y){
        double theta = ArmConstants.ARM_THETA;
        double r = ArmConstants.JOINT_LENGTH
        double tan_theta = Math.tan(theta);
        
        double a = (1 + tan_theta*tan_theta);
        double b = 2*(x + y * tan_theta);
        double c = y*y - r*r;

        double discriminant = b*b - 4*a*c;

        if(Math.abs(discriminant) < 1e-4){
            double x = -b/2/a;
            double y = x * tan_theta;
            return new double[][]{{x,y}};
        }
        if(discriminant < 0) return null

        double x1 = (-b + Math.sqrt(discriminant)) / 2 / a;
        double x2 = (-b - Math.sqrt(discriminant)) / 2 / a; 

        double y1 = x1*tan_theta;
        double y2 = x2*tan_theta;

        double xlim = ArmConstants.SHAFT_LENGTH;

        if((x1 < 0 || x1 > xlim) && (x2 < 0 || x2 > xlim)) return null;
        if(x1 < 0 || x1 > xlim) return double[][] {{x2,y2}};
        if(x2 < 0 || x2 > xlim) return double[][] {{x1,y1}}; 

        return new double[][] {{x1,y1}, {x2, y2}};
    }

    public void targetSet(double h, double theta1, double theta2){

        elevatorPID.setPID();
        armPID1.setPID();
        armPID2.setPID();

    }

    public static ArmSubsystem getInstance() {
        if(armSubsystem == null){
            this.armSubsystem = new ArmSubsystem();
        }

        return armSubsystem;

    } 
}   
