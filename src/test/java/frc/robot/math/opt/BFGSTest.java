package frc.robot.math.opt;

import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

public class BFGSTest {

    @Test
    public void rosenbrock() {
        BFGS bfgs = new BFGS();
        double a = 1.0;
        double b = 100.0;

        var actual = new DiffFunc() {

            @Override
            public double evaluate(double[] input) {
                double x = input[0];
                double y = input[1];
                return Math.pow(a - x, 2) + b * Math.pow(y - Math.pow(x, 2), 2);
            }

            @Override
            public double[] gradient(double[] input) {
                double x = input[0];
                double y = input[1];
                double dfdx = 2 * (x - 1) - 4 * b * (y - Math.pow(x, 2)) * x;
                double dfdy = 2 * b * (y - Math.pow(x, 2));
                return new double[] {dfdx, dfdy};
            }

        };
        var approx = new FiniteDifference(actual::evaluate, 1e-5);
        var init = new double[] {20.0, 0.0};

        bfgs.optimize(actual, init);

        assertTrue(bfgs.isSuccess());
        assertTrue(Math.abs(bfgs.getOptValue()[0] - a) < 1e-3);
        assertTrue(Math.abs(bfgs.getOptValue()[1] - Math.pow(a, 2)) < 1e-3);

        bfgs.optimize(approx, init);

        assertTrue(bfgs.isSuccess());
        assertTrue(Math.abs(bfgs.getOptValue()[0] - a) < 1e-3);
        assertTrue(Math.abs(bfgs.getOptValue()[1] - Math.pow(a, 2)) < 1e-3);
    }

}
