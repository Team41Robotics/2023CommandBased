package frc.robot;

public class HDrive {
        VexMotorCtrl bot_lef = new VexMotorCtrl("bl",0);
        VexMotorCtrl top_lef = new VexMotorCtrl("tl",1);
        VexMotorCtrl mid = new VexMotorCtrl("m",2);
        VexMotorCtrl top_rgt = new VexMotorCtrl("tr",3);
        VexMotorCtrl bot_rgt = new VexMotorCtrl("br",4);

        public void drive(double vx, double vy, double omega) {
                double vl, vr, vm;
                vl = vx - omega;
                vr = vx + omega;
                vm = vy;
                
                bot_lef.set(vl);
                top_lef.set(vl);
                mid.set(vm);
                top_rgt.set(vr);
                bot_rgt.set(vr);
        }
        public void FODrive(double vx, double vy, double omega, double angle){
                double vl, vr, vm, fm,sm;
                fm = Math.sin(angle)*vy+Math.cos(angle)*vy;
                sm = Math.cos(angle)*vy+Math.sin(angle)*vy;
                vl = fm - omega;
                vr = fm + omega;
                vm = sm;
                
                bot_lef.set(vl);
                top_lef.set(vl);
                mid.set(vm);
                top_rgt.set(vr);
                bot_rgt.set(vr);
        }
}
