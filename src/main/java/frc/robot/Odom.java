package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class Odom {
        ArrayList<Double> odom_times = new ArrayList<>();
        ArrayList<Transform2d> odom = new ArrayList<>();

        Odom() {
                odom.add(new Transform2d());
                odom_times.add(Timer.getFPGATimestamp());
        }

        double pt = Timer.getFPGATimestamp();
        double ptheta = Robot.imu.getYaw();
        public void robotPeriodic() {
                double t = Timer.getFPGATimestamp();
                double dt = t - pt;
                // fake odom with navx // FIXME do real odom ; current not used
                double vx = Robot.imu.getVelocityX();
                double vy = Robot.imu.getVelocityY();
                double theta = Robot.imu.getAngle() / 180 * Math.PI;
                double dtheta = theta - ptheta;
                ptheta = theta;

                Translation2d transl = new Translation2d(vx * dt, vy * dt);

                Transform2d trans = new Transform2d(transl, new Rotation2d(dtheta));
                odom.add(now().plus(trans));
                odom_times.add(Timer.getFPGATimestamp());
                pt = t;
        }

        public Transform2d now() {
                return odom.get(odom.size()-1);
        }

        public Transform2d get(double time) {
                if(odom_times.size() == 0) return new Transform2d();
                // binary search on nearest odom measurement and interpolate
                int l = 0;
                int r = odom_times.size()-1;
                while (l + 1 < r) {
                        int mid = l+(r-l)/2;
                        if(odom_times.get(mid) < time) l = mid;
                        else r = mid;
                }
                assert(r==l+1);
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
}
