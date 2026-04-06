package frc.robot.math.opt;

import java.util.function.DoubleUnaryOperator;

public class Bisection {

    public static final double bisection(DoubleUnaryOperator f, double min, double max) {
        double fMin = f.applyAsDouble(min);
        double fMax = f.applyAsDouble(max);

        assert fMax * fMin < 0.0;

        for (int i = 0; i < 100; i++) {
            double mid = (min + max) / 2.0;
            double fMid = f.applyAsDouble(mid);
            double det = fMid * fMin;
            if (Math.abs(det) < 1e-3) {
                return mid;
            } else if (det < 0.0) {
                max = mid;
                fMax = fMid;
            } else {
                min = mid;
                fMin = fMid;
            }
        }
        return (min + max) / 2.0;
    }

}
