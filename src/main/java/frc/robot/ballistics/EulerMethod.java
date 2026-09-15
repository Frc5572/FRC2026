package frc.robot.ballistics;

import java.util.ArrayList;
import java.util.List;

public class EulerMethod {
    public static List<double[]> solveIVP(DerivableFunction f, double[] x0, int steps, double dt) {
        List<double[]> ret = new ArrayList<>();
        for (int i = 0; i < steps; i++) {
            double[] tempX = new double[x0.length];
            double[] fX = f.derivative(x0);

            for (int j = 0; j < x0.length; j++) {
                tempX[j] = x0[j] + fX[j] * dt;
            }

            x0 = tempX;
            ret.add(tempX);
        }

        return ret;
    }
}
