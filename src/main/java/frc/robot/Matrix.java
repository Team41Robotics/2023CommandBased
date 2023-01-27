package frc.robot;

public class Matrix {
        int n, m;
        double[][] mat;

        public Matrix(double[][] mat) {
                this.n = mat.length;
                this.m = mat[0].length;
                this.mat = mat;
        }

        public void print() {
                for (int i = 0; i < n; i++) {
                        for (int j = 0; j < m; j++) {
                                if (mat[i][j] > 999) System.out.print("INF");
                                else if (mat[i][j] < -999) System.out.print("-INF");
                                else System.out.print(Math.round(mat[i][j] * 1000) / 1000.);
                                System.out.print("\t");
                        }
                        System.out.println("");
                }
        }

        public static Matrix zero(int n, int m) {
                Matrix M = new Matrix(new double[n][m]);
                for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) M.mat[i][j] = 0;
                return M;
        }

        public static Matrix eye(int n) {
                Matrix M = new Matrix(new double[n][n]);
                for (int i = 0; i < n; i++)
                        for (int j = 0; j < n; j++)
                                if (i == j) M.mat[i][j] = 1;
                                else M.mat[i][j] = 0;
                return M;
        }

        public Matrix mul(Matrix o) {
                if (m != o.n) return null;
                Matrix prod = Matrix.zero(n, o.m);
                for (int i = 0; i < n; i++)
                        for (int j = 0; j < o.m; j++)
                                for (int k = 0; k < m; k++) prod.mat[i][j] += mat[i][k] * o.mat[k][j];
                return prod;
        }

        public Matrix T() {
                Matrix T = new Matrix(new double[m][n]);
                for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) T.mat[j][i] = mat[i][j];
                return T;
        }

        public Matrix inv() {
                if (n != m) return null;
                Matrix aug = new Matrix(new double[n][2 * n]);
                for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) aug.mat[i][j] = mat[i][j];

                for (int i = 0; i < n; i++)
                        for (int j = 0; j < n; j++)
                                if (i == j) aug.mat[i][j + n] = 1;
                                else aug.mat[i][j + n] = 0;

                for (int i = 0; i < n; i++) {
                        int piv = i;
                        double elem = aug.mat[i][i];
                        for (int j = i; j < n; j++) { // argmin
                                if (aug.mat[j][i] > elem) {
                                        piv = j;
                                        elem = aug.mat[j][i];
                                }
                        }
                        for (int j = 0; j < n * 2; j++) {
                                // swap aug.mat[i][j], aug.mat[piv][j]
                                double tmp = aug.mat[i][j];
                                aug.mat[i][j] = aug.mat[piv][j];
                                aug.mat[piv][j] = tmp;
                        }
                        // eliminate
                        for (int j = i + 1; j < n; j++) {
                                double fac = -aug.mat[j][i] / elem;
                                for (int k = 0; k < 2 * n; k++) aug.mat[j][k] += aug.mat[i][k] * fac;
                        }
                }
                // left side is upper rectangular right now
                for (int i = n - 1; i >= 0; i--) {
                        double elem = aug.mat[i][i];
                        for (int j = i - 1; j >= 0; j--) {
                                double fac = -aug.mat[j][i] / elem;
                                for (int k = 0; k < 2 * n; k++) aug.mat[j][k] += aug.mat[i][k] * fac;
                        }
                }
                for (int i = 0; i < n; i++) {
                        double fac = 1 / aug.mat[i][i];
                        for (int k = 0; k < 2 * n; k++) aug.mat[i][k] *= fac;
                }
                Matrix ret = new Matrix(new double[n][n]);
                for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) ret.mat[i][j] = aug.mat[i][j + n];
                return ret;
        }
}
