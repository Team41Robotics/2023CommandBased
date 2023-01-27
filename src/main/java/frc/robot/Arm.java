package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
 


public class Arm {

    private static Encoder motor1Encoder;
    private static Encoder motor2Encoder;
    private static Encoder motor3Encoder;

    private static VexMotorCtrl motor1;
    private static VexMotorCtrl motor2;
    private static VexMotorCtrl motor3;

    private static final int  length1 = 10;
    private static final int length2 = 5;
    private static final int length3 = 100;

    private static boolean engaged;

    private static DigitalInput originLimit;
    private static DigitalInput maximumLimit;

    public void initArm(){

        motor1Encoder = new Encoder(0,1,true);
        motor2Encoder = new Encoder(2,3,true);
        motor3Encoder = new Encoder(4,5,true);

        motor1 = new VexMotorCtrl("motor1", 1);
        motor2 = new VexMotorCtrl("motor2", 2);
        motor3 = new VexMotorCtrl("motor3", 3);
        
        originLimit = new DigitalInput(0);
        maximumLimit = new DigitalInput(1);

        motor1Encoder.reset();
        motor2Encoder.reset();
        motor3Encoder.reset();

    }

    public void 


}
