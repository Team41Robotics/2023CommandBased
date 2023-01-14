package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class HDrive {
        ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

        VexMotorCtrl bot_lef = new VexMotorCtrl("bl",0);
        VexMotorCtrl top_lef = new VexMotorCtrl("tl",1);
        VexMotorCtrl mid = new VexMotorCtrl("m",2);
        VexMotorCtrl top_rgt = new VexMotorCtrl("tr",3);
        VexMotorCtrl bot_rgt = new VexMotorCtrl("br",4);

        public HDrive() {
                dttab.addNumber("vl",()->vl);
                dttab.addNumber("vr",()->vr);
                dttab.addNumber("vm",()->vm);
        }

        double vl, vr, vm;
        public void drive(double vx, double vy, double omega) {
                vl = vx - omega;
                vr = vx + omega;
                vm = vy;

                bot_lef.set(vl);
                top_lef.set(vl);
                mid.set(vm);
                top_rgt.set(vr);
                bot_rgt.set(vr);
        }
}
