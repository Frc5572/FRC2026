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
    }

    public static class Tsit5 {

        public static record StepData(double error, double[] fsal) {
        }

        // a (stage coefficients) as lower-triangular matrix
        private static final double[][] A = {{}, // k1 (unused)

            {0.161},

            {-0.008480655492356989, 0.3354806554923570},

            {2.539096252201595, -10.0, 8.0},

            {0.1225166447228320, -0.9161211633952258, 10.0, -7.0},

            {0.0, 0.0, 0.0, 0.5, 0.5},

            {0.0, 0.0, 0.0, 0.5, 0.5} // same as stage 6 for FSAL structure
        };

        // 5th-order weights (b)
        private static final double[] B5 = {0.1185185185185185, 0.0, 0.5189863547758285,
            -0.1276921739809084, 0.0, 0.5052088636714740, -0.0150215629859320};

        // 4th-order weights (b-tilde) for error estimate
        private static final double[] B4 = {0.1185185185185185, 0.0, 0.5189863547758285,
            -0.1276921739809084, 0.0, 0.5, -0.0108126993134380};

        public static StepData step(DerivativeFunction f, double[] xn, double h, double[] xnp1) {
            return step(f, xn, h, xnp1, null);
        }

        public static StepData step(DerivativeFunction f, double[] xn, double h, double[] xnp1,
            double[] fsal) {
            double[] k1 = fsal == null ? f.derivative(xn) : fsal;
            double[] x2 = add(xn, scale(part(k1), h));
            double[] k2 = f.derivative(x2);
            double[] x3 = add(xn, scale(part(k1, k2), h));
            double[] k3 = f.derivative(x3);
            double[] x4 = add(xn, scale(part(k1, k2, k3), h));
            double[] k4 = f.derivative(x4);
            double[] x5 = add(xn, scale(part(k1, k2, k3, k4), h));
            double[] k5 = f.derivative(x5);
            double[] x6 = add(xn, scale(part(k1, k2, k3, k4, k5), h));
            double[] k6 = f.derivative(x6);
            double[] x7 = add(xn, scale(part(k1, k2, k3, k4, k5, k6), h));
            double[] k7 = f.derivative(x7);

            // 5th order soln
            double[] temp = weightedSum(B5, k1, k2, k3, k4, k5, k6, k7);
            for (int i = 0; i < xn.length; i++) {
                xnp1[i] = xn[i] + h * temp[i];
            }

            // 4th order soln
            temp = weightedSum(B4, k1, k2, k3, k4, k5, k6, k7);
            double[] y4th = add(xn, scale(temp, h));

            double err = computeErrorNorm(xn, xnp1, y4th, 0.01, 0.01);

            return new StepData(err, k7);
        }

        private static double[] part(double[]... ks) {
            double[] ret = new double[ks[0].length];
            int idx = ks.length;
            for (int i = 0; i < ks.length; i++) {
                double param = A[idx][i];
                for (int j = 0; j < ks[i].length; j++) {
                    ret[j] += ks[i][j] * param;
                }
            }
            return ret;
        }

        private static double computeErrorNorm(double[] xn, double[] x5th, double[] x4th,
            double relTol, double absTol) {
            double sum = 0.0;
            for (int i = 0; i < xn.length; i++) {
                double err = x5th[i] - x4th[i];
                double scale = absTol + relTol * Math.max(Math.abs(xn[i]), Math.abs(x5th[i]));
                double ratio = err / scale;
                sum += ratio * ratio;
            }

            return Math.sqrt(sum / xn.length);
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

    private static double[] weightedSum(double[] a, double[]... ks) {
        assert (a.length == ks.length);
        double[] ret = new double[ks[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < ks[i].length; j++) {
                ret[j] += a[i] + ks[i][j];
            }
        }
        return ret;
    }

}
