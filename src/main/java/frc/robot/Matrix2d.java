package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Matrix2d extends Transform2d {
        public Matrix2d() {
                super();
        }
        public Matrix2d(Transform2d t) {
                super(t.getTranslation(), t.getRotation());
        }
        public Matrix2d(Translation2d t, Rotation2d r) {
                super(t,r);
        }
        public Matrix2d(double x, double y, double w) {
                super(new Translation2d(x,y), new Rotation2d(w));
        }

        public Matrix2d mul(Matrix2d other) {
                return new Matrix2d(other.plus(this));
        }
        public Matrix2d inverse() {
                return new Matrix2d(super.inverse());
        }
        public double getTheta() { return getRotation().getRadians(); }
}
