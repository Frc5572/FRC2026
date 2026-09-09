package frc.robot.ballistics;

public class Integrators {

    public static class Euler {

        public static void step(DerivativeFunction f, double[] xn, double h, double[] xnp1) {
            double[] fx = f.derivative(xn);
            for (int i = 0; i < xn.length; i++) {
                xnp1[i] = xn[i] + fx[i] * h;
            }
        }

    }

    public static class RK4 {

        public static void step(DerivativeFunction f, double[] xn, double h, double[] xnp1) {
            double[] k1 = f.derivative(xn);
            double[] x2 = add(xn, scale(k1, h / 2.0));
            double[] k2 = f.derivative(x2);
            double[] x3 = add(xn, scale(k2, h / 2.0));
            double[] k3 = f.derivative(x3);
            double[] x4 = add(xn, scale(k3, h));
            double[] k4 = f.derivative(x4);

            for (int i = 0; i < xn.length; i++) {
                xnp1[i] = xn[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
        }

        private static double[] add(double[] a, double[] b) {
            assert (a.length == b.length);
            double[] ret = new double[a.length];
            for (int i = 0; i < a.length; i++) {
                ret[i] = a[i] + b[i];
            }
            return ret;
        }



        private static double[] scale(double[] a, double b) {
            double[] ret = new double[a.length];
            for (int i = 0; i < a.length; i++) {
                ret[i] = a[i] * b;
            }
            return ret;
        }
    }

}
