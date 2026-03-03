package frc.robot.math.interp2d;

import java.util.function.DoubleUnaryOperator;

public class Rbfs {

    public static DoubleUnaryOperator gaussian(final double eps) {
        return (r) -> Math.exp(-(eps * r) * (eps * r));
    }

    public static DoubleUnaryOperator inverseQuadratic(final double eps) {
        return (r) -> 1.0 / (1.0 + (eps * r) * (eps * r));
    }

    public static DoubleUnaryOperator inverseMultiquadric(final double eps) {
        return (r) -> 1.0 / Math.sqrt(1.0 + (eps * r) * (eps * r));
    }

    public static DoubleUnaryOperator multiquadric(final double eps) {
        return (r) -> Math.sqrt(1.0 + (eps * r) * (eps * r));
    }

    public static DoubleUnaryOperator rth(final double eps) {
        return (r) -> r * Math.tanh(eps * r);
    }

    public static DoubleUnaryOperator polyharmonicSpline(final int k) {
        if (k % 2 == 1) {
            return (r) -> Math.pow(r, k);
        } else {
            return (r) -> {
                if (r < 1e-9) {
                    r = 1e-9;
                }
                return Math.pow(r, k) * Math.log(r);
            };
        }
    }

    public static DoubleUnaryOperator thinPlate() {
        return polyharmonicSpline(2);
    }

    public static DoubleUnaryOperator bump(final double eps) {
        return (r) -> {
            if (r < 1.0 / eps) {
                return Math.exp(-(1.0 / (1.0 - (eps * r) * (eps * r))));
            } else {
                return 0.0;
            }
        };
    }
}
