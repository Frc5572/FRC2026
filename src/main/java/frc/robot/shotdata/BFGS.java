package frc.robot.shotdata;

import java.util.function.ToDoubleFunction;

public class BFGS {

    /**
     * Find x such that f(x) is minimized. Finds f'(x) via finite difference. Only guaranteed to
     * find the correct answer if f(x) is convex.
     */
    public static double[] minimize(double[] x0, ToDoubleFunction<double[]> f) {
        return minimize(x0, f, 1e-6, 100);
    }

    /**
     * Find x such that f(x) is minimized. Finds f'(x) via finite difference. Only guaranteed to
     * find the correct answer if f(x) is convex.
     */
    public static double[] minimize(double[] x0, ToDoubleFunction<double[]> f, double tol) {
        return minimize(x0, f, tol, 100);
    }

    /**
     * Find x such that f(x) is minimized. Finds f'(x) via finite difference. Only guaranteed to
     * find the correct answer if f(x) is convex.
     */
    public static double[] minimize(double[] x0, ToDoubleFunction<double[]> f, double tol,
        int maxItr) {
        // TODO https://github.com/wilsonwatson/matlib/blob/master/matlib/bfgs.h
        return x0;
    }

    private static final double DX = 0.01;

    private double[] gradient(double[] x, ToDoubleFunction<double[]> f) {
        double[] diff = new double[x.length];
        double[] res = new double[x.length];
        System.arraycopy(x, 0, diff, 0, x.length);
        double base = f.applyAsDouble(x);
        for (int i = 0; i < x.length; i++) {
            diff[i] = x[i] + DX;
            double nudge = f.applyAsDouble(diff);
            res[i] = (nudge - base) / DX;
            diff[i] = x[i];
        }
        return res;
    }

}
