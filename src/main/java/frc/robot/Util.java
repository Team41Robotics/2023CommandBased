package frc.robot;

public class Util {
        public static double deadZone(double joystickAxis) {
                return Math.abs(joystickAxis) > 0.2 ? joystickAxis : 0;
        }

        public static Matrix interpolate(Matrix a, Matrix b, double alpha) {
                // linear interpolate; (1-alpha) a + alpha b
                double ra = a.getTheta();
                double rb = b.getTheta();
                double ro = ra + normRot(rb - ra) * alpha;

                //Translation2d ta = a.getTranslation();
                //Translation2d tb = b.getTranslation();
                //Translation2d to = ta.times(1 - alpha).plus(tb.times(alpha));
                double tx = a.getX() + b.getX();
                double ty = a.getY() + b.getY();
                return Matrix.create(tx,ty,ro);
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