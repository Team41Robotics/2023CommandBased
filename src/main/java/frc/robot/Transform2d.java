package frc.robot;

public class Transform2d {
    Matrix mat;

    public Transform2d() {
        mat =
                new Matrix(
                        new double[][] {
                            {1, 0, 0},
                            {0, 1, 0},
                            {0, 0, 1}
                        });
    }

    public Transform2d(Matrix mat) {
        this.mat = mat;
    }

    public Transform2d(double x, double y, double theta) {
        mat =
                new Matrix(
                        new double[][] {
                            {Math.cos(theta), -Math.sin(theta), x},
                            {Math.sin(theta), Math.cos(theta), y},
                            {0, 0, 1}
                        });
    }

    public void print() { mat.print(); }

    public Transform2d mul(Transform2d other) {
        return new Transform2d(mat.mul(other.mat));
    }

    public Transform2d inv() {
        return new Transform2d(mat.inv());
    }

    public double getX() {
        return mat.mat[0][2];
    }

    public double getY() {
        return mat.mat[1][2];
    }

    public double getTheta() {
        return Math.atan2(mat.mat[1][0], mat.mat[0][0]);
    }
}
