package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Transform2d;
import frc.robot.Util;
import java.util.ArrayList;

public class OdomSubsystem extends SubsystemBase {
    ArrayList<Double> times = new ArrayList<>();
    ArrayList<Transform2d> odoms = new ArrayList<>();
    Transform2d odom_origin = new Transform2d(2, 3, Math.PI / 2);

    static OdomSubsystem odom;

    ShuffleboardTab odomstab = Shuffleboard.getTab("Odom");
    HDriveSubsystem hdrive = HDriveSubsystem.getInstance();

    public OdomSubsystem() {
        times.ensureCapacity(7000);
        odoms.ensureCapacity(7000);
        odomstab.addNumber("x", () -> now().getX());
        odomstab.addNumber("y", () -> now().getY());
        odomstab.addNumber("theta", () -> now().getTheta());
        odomstab.addNumber("ox", () -> odom_origin.getX());
        odomstab.addNumber("oy", () -> odom_origin.getY());
        odomstab.addNumber("otheta", () -> odom_origin.getTheta());
        odomstab.addNumber("Mx", () -> odoms.get(odoms.size() - 1).getX());
        odomstab.addNumber("My", () -> odoms.get(odoms.size() - 1).getY());
        odomstab.addNumber("Mtheta", () -> odoms.get(odoms.size() - 1).getTheta());
        odoms.add(new Transform2d(0, 0, 0));
        times.add(Timer.getFPGATimestamp());
    }

    public boolean isStarted = false;

    public void start() {
        isStarted = true;
        ptheta = Robot.imu.getAngle();
    }

    double ptheta = Robot.imu.getAngle();
    double pl_enc = hdrive.lef_enc.getDistance();
    double pm_enc = hdrive.rgt_enc.getDistance();
    double pr_enc = hdrive.mid_enc.getDistance();

    @Override
    public void periodic() {
        if (!isStarted) return;
        double theta = Robot.imu.getAngle();
        double dtheta = theta - ptheta;
        ptheta = theta;

        double lenc = hdrive.lef_enc.getDistance();
        double menc = hdrive.mid_enc.getDistance();
        double renc = hdrive.rgt_enc.getDistance();
        double dl = lenc - pl_enc;
        double ds = menc - pm_enc;
        double dr = renc - pr_enc;
        pl_enc = lenc;
        pm_enc = menc;
        pr_enc = renc;
        double df = (dl + dr) / 2;

        double dx, dy;
        if (dtheta < 1e-9) {
            dx = df;
            dy = ds;
        } else {
            dx = Math.sin(dtheta) / dtheta * df + (Math.cos(dtheta) - 1) / dtheta * ds;
            dy = (1 - Math.cos(dtheta)) / dtheta * df + Math.sin(dtheta) / dtheta * ds;
        }


        Transform2d trans = new Transform2d(dx, dy, dtheta);
        Transform2d acc = odoms.get(odoms.size() - 1);
        odoms.add(acc.mul(trans));
        times.add(Timer.getFPGATimestamp());
    }

    public Transform2d now() {
        // return odoms.get(odoms.size() - 1);
        return odom_origin.mul(odoms.get(odoms.size() - 1));
    }

    public Transform2d get(double time) {
        if (times.size() == 0) return new Transform2d(0, 0, 0);
        // binary search on nearest odoms measurement and interpolate
        int l = 0;
        int r = times.size() - 1;
        while (l < r) {
            int mid = l + (r - l) / 2;
            if (times.get(mid) < time) l = mid + 1;
            else r = mid;
        }
        // r is the index of the odoms measurement that is after time
        l = r - 1;
        if (l < 0) return odoms.get(0);
        System.out.println("l = " + l);
        Transform2d left = odoms.get(l);
        Transform2d right = odoms.get(r);
        double ltime = times.get(l);
        double rtime = times.get(r);
        //return odom_origin.mul(Util.interpolate(left, right, (time - ltime) / (rtime - ltime)));
        return odom_origin.mul(left);
    }

    public Transform2d delta(double time) {
        // now is Tn... T3 T2 T1 T0
        // get(x) Tx... T3 T2 T1 T0
        // we want now * get(x)inv
        return now().mul(get(time).inv());
    }

    public void update_from(Transform2d pose, double time) {
        System.out.println("x " + pose.getX() + " y " + pose.getY() + " theta " + pose.getTheta());
        Transform2d acc = odom_origin.inv().mul(get(time));
        // odom_origin * acc = pose
        // odom_origin = pose * acc^-1
        odom_origin = pose.mul(acc.inv());
    }

    public static OdomSubsystem getInstance() {
        if (odom == null) {
            odom = new OdomSubsystem();
        }
        return odom;
    }
}
