package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class Util {
        public static double deadZone(double joystickAxis) {
                return Math.abs(joystickAxis) > 0.2 ? joystickAxis : 0;
        }

        public static Matrix2d transformThreeToTwo(Transform3d trans) {
                return new Matrix2d(trans.getTranslation().toTranslation2d(), trans.getRotation().toRotation2d());
        }

        public static Matrix2d interpolate(Matrix2d a, Matrix2d b, double alpha) {
                // linear interpolate; (1-alpha) a + alpha b
                double ra = a.getTheta();
                double rb = b.getTheta();
                double ro = ra + normRot(rb - ra) * alpha;

                Translation2d ta = a.getTranslation();
                Translation2d tb = b.getTranslation();
                Translation2d to = ta.times(1 - alpha).plus(tb.times(alpha));
                return new Matrix2d(to, new Rotation2d(ro));
        }

        public static double normRot(double rad) {
                rad %= 2 * Math.PI;
                rad += 2 * Math.PI;
                rad %= 2 * Math.PI;
                if (rad > Math.PI)
                        rad -= 2 * Math.PI;
                return rad;
        }
}