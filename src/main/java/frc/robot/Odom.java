package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// FIXME UNTESTED
public class Odom {
        ArrayList<Double> odom_times = new ArrayList<>();
        ArrayList<Transform2d> odom = new ArrayList<>();

        ShuffleboardTab odomtab = Shuffleboard.getTab("Odom");

        Odom() {
                odomtab.addNumber("x", () -> now().getX());
                odomtab.addNumber("y", () -> now().getY());
                odomtab.addNumber("theta", () -> now().getRotation().getDegrees());
                odom.add(new Transform2d());
                odom_times.add(Timer.getFPGATimestamp());
        }

        double ptheta = Robot.imu.getAngle();
        double pl_enc = HDrive.lef_enc.getDistance();
        double pm_enc = HDrive.rgt_enc.getDistance();
        double pr_enc = HDrive.mid_enc.getDistance();
        public void robotPeriodic() {
                double theta = Robot.imu.getAngle();
                double dtheta = theta - ptheta;
                ptheta = theta;

                double lenc = HDrive.lef_enc.getDistance();
                double menc = HDrive.mid_enc.getDistance();
                double renc = HDrive.rgt_enc.getDistance();
                double dl = lenc-pl_enc;
                double ds = menc-pm_enc;
                double dr = renc-pr_enc;
                pl_enc = lenc;
                pm_enc = menc;
                pr_enc = renc;
                double df = (dl + dr) / 2;

                double dx, dy;
                if(dtheta < 1e9) {
                        dx = df; dy = ds;
                }
                else {
                        dx = Math.sin(dtheta)/dtheta * df + (Math.cos(dtheta)-1)/dtheta * ds;
                        dy = (1-Math.sin(dtheta))/dtheta * df + Math.sin(dtheta)/dtheta * ds;
                }

                Transform2d trans = new Transform2d(new Translation2d(dx,dy), new Rotation2d(dtheta));
                odom.add(now().plus(trans));
                odom_times.add(Timer.getFPGATimestamp());
        }

        public Transform2d now() {
                return odom.get(odom.size()-1);
        }

        public Transform2d get(double time) {
                if(odom_times.size() == 0) return new Transform2d();
                // binary search on nearest odom measurement and interpolate
                int l = 0;
                int r = odom_times.size()-1;
                while (l < r) {
                        int mid = l+(r-l)/2;
                        if(odom_times.get(mid) < time) l = mid + 1;
                        else r = mid;
                }
                // r is the index of the odom measurement that is after time
                l = r-1;
                if(l<0) return odom.get(0);
                Transform2d left = odom.get(l);
                Transform2d right = odom.get(r);
                double ltime = odom_times.get(l);
                double rtime = odom_times.get(r);
                return Util.interpolate(left, right, (time - ltime) / (rtime - ltime));
        }

        public Transform2d delta(double time) {
                // now is Tn... T3 T2 T1 T0
                // get(x) Tx... T3 T2 T1 T0
                // we want now * get(x)inv
                return get(time).inverse().plus(now());
        }

        public ArrayList<Double> getOdom_times() {
                return odom_times;
        }

        public void setOdom_times(ArrayList<Double> odom_times) {
                this.odom_times = odom_times;
        }

        public ArrayList<Transform2d> getOdom() {
                return odom;
        }

        public void setOdom(ArrayList<Transform2d> odom) {
                this.odom = odom;
        }

        public ShuffleboardTab getOdomtab() {
                return odomtab;
        }

        public void setOdomtab(ShuffleboardTab odomtab) {
                this.odomtab = odomtab;
        }

        public double getPtheta() {
                return ptheta;
        }

        public void setPtheta(double ptheta) {
                this.ptheta = ptheta;
        }

        public double getPl_enc() {
                return pl_enc;
        }

        public void setPl_enc(double pl_enc) {
                this.pl_enc = pl_enc;
        }

        public double getPm_enc() {
                return pm_enc;
        }

        public void setPm_enc(double pm_enc) {
                this.pm_enc = pm_enc;
        }

        public double getPr_enc() {
                return pr_enc;
        }

        public void setPr_enc(double pr_enc) {
                this.pr_enc = pr_enc;
        }
}
