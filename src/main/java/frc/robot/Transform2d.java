package frc.robot;

public class Transform2d extends Matrix {
        public Transform2d() {
                super(
                                new double[][] {
                                        {1, 0, 0},
                                        {0, 1, 0},
                                        {0, 0, 1}
                                });
        }

        public Transform2d(Matrix mat) {
                super(mat.mat);
        }

        public Transform2d(double x, double y, double theta) {
                super(
                                new double[][] {
                                        {Math.cos(theta), -Math.sin(theta), x},
                                        {Math.sin(theta), Math.cos(theta), y},
                                        {0, 0, 1}
                                });
        }

        public Transform2d mul(Transform2d other) {
                return new Transform2d(super.mul(other));
        }

        public Transform2d inv() {
                return new Transform2d(super.inv());
        }

        public double getX() {
                return mat[0][2];
        }

        public double getY() {
                return mat[1][2];
        }

        public double getTheta() {
                return Math.atan2(mat[1][0], mat[0][0]);
        }
}
