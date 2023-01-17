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

        boolean FOD = false;
        public void teleopPeriodic() {
                double vf = -Util.deadZone(DS.getLY());
                double vs = -Util.deadZone(DS.getLX());
                double omega = Util.deadZone(DS.getRX());

                if(Math.abs(omega) < 0.5) omega=0;

                double robot_angle = Robot.imu.getYaw();
                double vx = vf * Math.cos(robot_angle*Math.PI/180) - vs * Math.sin(robot_angle*Math.PI/180);
                double vy = vf * Math.sin(robot_angle*Math.PI/180) + vs * Math.cos(robot_angle*Math.PI/180);

                if(DS.right_js.getRawButtonPressed(2)) FOD = !FOD;
                if(DS.left_js.getRawButton(2)) Robot.imu.zeroYaw();

                if(DS.getLTrig()) // janky; FIXME later
                        drive(0.5 * Math.signum(Robot.imu.getPitch())*Math.sqrt(Math.abs(Robot.imu.getPitch() / 15)),0,0);
                else if(FOD) 
                        drive(-vx, vy,-omega);
                else
                        drive(vf, -vs, -omega);
        }
}
