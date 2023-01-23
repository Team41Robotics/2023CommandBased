package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VexMotorCtrl;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DrivetrainConstants;
import static frc.robot.Constants.DrivetrainConstants;
public class HDriveSubsystem extends SubsystemBase {

        ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

        public Encoder lef_enc = new Encoder(0, 1);
        public Encoder mid_enc = new Encoder(2, 3);
        public Encoder rgt_enc = new Encoder(4, 5);
        
        static HDriveSubsystem hdrive;
        public boolean FOD;

        VexMotorCtrl bot_lef = new VexMotorCtrl("bl", DrivetrainConstants.BACK_LEFT_MOTOR_CHANNEL);
        VexMotorCtrl top_lef = new VexMotorCtrl("tl",1);
        VexMotorCtrl mid = new VexMotorCtrl("m",2);
        VexMotorCtrl top_rgt = new VexMotorCtrl("tr",3);
        VexMotorCtrl bot_rgt = new VexMotorCtrl("br",4);

        double vl, vr, vm;

        public HDriveSubsystem() {
                super();
                lef_enc.setDistancePerPulse(0.05 * 2*Math.PI/90);
                mid_enc.setDistancePerPulse(0.05 * 2*Math.PI/90);
                rgt_enc.setDistancePerPulse(0.05 * 2*Math.PI/90);
                dttab.addNumber("le", ()->lef_enc.getDistance());
                dttab.addNumber("me", ()->mid_enc.getDistance());
                dttab.addNumber("re", ()->rgt_enc.getDistance());

                dttab.addNumber("vl",()->vl);
                dttab.addNumber("vr",()->vr);
                dttab.addNumber("vm",()->vm);

        }
        
        public void drive(double vx, double vy, double omega) {
            vl = -vx + omega;
            vr = -vx - omega; 
            vm = vy;

            bot_lef.set(vl);
            top_lef.set(vl);
            mid.set(vm);
            top_rgt.set(vr);
            bot_rgt.set(vr);

        }

        public static HDriveSubsystem getInstance() {
            if(hdrive == null){
                 hdrive = new HDriveSubsystem();
            }
            return hdrive;
        }
}