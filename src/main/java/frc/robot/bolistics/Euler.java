package frc.robot.bolistics;

import java.util.ArrayList;
import java.util.List;

public class Euler {
    public static List<double[]> solveIVP(DerivableFunction f, double[] x0, int steps, double dt) {
        List<double[]> ret = new ArrayList<>();

        for (int i = 0; i < steps; i++) {
            double[] temp = new double[x0.length];
            double[] fx = f.derivative(x0);
            for (int j = 0; j < x0.length; j++) {
                temp[j] = x0[j] + fx[j] * dt;
            }
            x0 = temp;
            ret.add(temp);
        }
        return ret;
    }
}
