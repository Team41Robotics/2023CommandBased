package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VexMotorCtrl;

public class HDriveSubsystem extends SubsystemBase {
        public ShuffleboardTab dttab = Shuffleboard.getTab("Drivetrain");
        public Encoder lef_enc = new Encoder(0, 1, true);
        public Encoder mid_enc = new Encoder(2, 3);
        public Encoder rgt_enc = new Encoder(4, 5);

        static HDriveSubsystem hdrive;

        VexMotorCtrl bot_lef = new VexMotorCtrl("bl", 0);
        VexMotorCtrl top_lef = new VexMotorCtrl("tl", 1);
        VexMotorCtrl mid = new VexMotorCtrl("m", 2);
        VexMotorCtrl top_rgt = new VexMotorCtrl("tr", 3);
        VexMotorCtrl bot_rgt = new VexMotorCtrl("br", 4);

        double vl, vr, vm;
        double vx, vy, w;

        public HDriveSubsystem() {
                super();
                lef_enc.setDistancePerPulse(0.05 * 2 * Math.PI / 90);
                mid_enc.setDistancePerPulse(0.05 * 2 * Math.PI / 90);
                rgt_enc.setDistancePerPulse(0.05 * 2 * Math.PI / 90);

                dttab.add(this);
                dttab.addNumber("le", () -> lef_enc.getDistance());
                dttab.addNumber("me", () -> mid_enc.getDistance());
                dttab.addNumber("re", () -> rgt_enc.getDistance());

                dttab.addNumber("vl", () -> vl);
                dttab.addNumber("vr", () -> vr);
                dttab.addNumber("vm", () -> vm);

                dttab.addNumber("vx", () -> vx);
                dttab.addNumber("vy", () -> vy);
                dttab.addNumber("w", () -> w);
        }

        public void drive(double vx, double vy, double w) {
                drive(vx, vy, w, true);
        }

        public void drive(double vx, double vy, double w, boolean preserve) {
                this.vx = vx;
                this.vy = vy;
                this.w = w;
                vl = -vx + w;
                vr = -vx - w;
                vm = vy;

                double max = Math.max(Math.max(Math.abs(vl), Math.abs(vm)), Math.abs(vr));
                if (max > 1) {
                        vx /= max;
                        vy /= max;
                        w /= max;
                        vl = -vx + w;
                        vr = -vx - w;
                        vm = vy;
                }

                bot_lef.set(vl);
                top_lef.set(vl);
                mid.set(vm);
                top_rgt.set(vr);
                bot_rgt.set(vr);
        }

        public static HDriveSubsystem getInstance() {
                if (hdrive == null) {
                        hdrive = new HDriveSubsystem();
                }
                return hdrive;
        }
}
