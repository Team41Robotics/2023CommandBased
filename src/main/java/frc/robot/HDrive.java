package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class HDrive {
        ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");

        VexMotorCtrl bot_lef = new VexMotorCtrl("bl",0);
        VexMotorCtrl top_lef = new VexMotorCtrl("tl",1);
        VexMotorCtrl mid = new VexMotorCtrl("m",2);
        VexMotorCtrl top_rgt = new VexMotorCtrl("tr",3);
        VexMotorCtrl bot_rgt = new VexMotorCtrl("br",4);

        // VEX Encoders: 90 ticks = 2PI;
        static Encoder lef_enc = new Encoder(0, 1);
        static Encoder mid_enc = new Encoder(2,3);
        static Encoder rgt_enc = new Encoder(4,5, true);

        public HDrive() {
                lef_enc.setDistancePerPulse(0.05 * 2*Math.PI/90);
                mid_enc.setDistancePerPulse(0.05 * 2*Math.PI/90);
                rgt_enc.setDistancePerPulse(0.05 * 2*Math.PI/90);
                dttab.addNumber("le", ()->lef_enc.getDistance());
                dttab.addNumber("me", ()->mid_enc.getDistance());
                dttab.addNumber("re", ()->rgt_enc.getDistance());

                dttab.addNumber("vl",()->vl);
                dttab.addNumber("vr",()->vr);
                dttab.addNumber("vm",()->vm);

                dttab.addBoolean("FOD",()->FOD);
        }

        double vl, vr, vm;
        public void drive(double vx, double vy, double omega) {
                vl = vx + omega;
                vr = vx - omega;
                vm = vy;

                bot_lef.set(vl);
                top_lef.set(vl);
                mid.set(vm);
                top_rgt.set(vr);
                bot_rgt.set(vr);
        }

        public void FODdrive(double vf, double vs, double omega) {
                double robot_angle = Robot.imu.getYaw();
                double vx = vf * Math.cos(robot_angle) - vs * Math.sin(robot_angle);
                double vy = vf * Math.sin(robot_angle) + vs * Math.cos(robot_angle);

                drive(vx, vy, omega);
        }

        boolean FOD = false;
        public void teleopPeriodic() {
                double vf = -Util.deadZone(DS.getLY());
                double vs = -Util.deadZone(DS.getLX());
                double omega = -Util.deadZone(DS.getRX());

                if(Math.abs(omega) < 0.3) omega=0;

                if(DS.right_js.getRawButtonPressed(2)) FOD = !FOD;
                if(DS.left_js.getRawButton(2)) Robot.imu.zeroYaw();

                //if(DS.getLTrig()) // janky; FIXME later actually doesn't work anymore
                        //drive(0.5 * Math.signum(Robot.imu.getPitch())*Math.sqrt(Math.abs(Robot.imu.getPitch() / 15)),0,0);
                else if(FOD) 
                        FODdrive(vf, vs, omega);
                else
                        drive(vf, vs, omega);
        }
}
