package frc.robot.math.opt;

import java.util.function.ToDoubleFunction;

/** Create a differentiable function by way of finite difference. */
public class FiniteDifference implements DiffFunc {

    private final ToDoubleFunction<double[]> func;
    private final double h;

    /** Create a differentiable function by way of finite difference. */
    public FiniteDifference(ToDoubleFunction<double[]> func, double h) {
        this.func = func;
        this.h = h;
    }

    @Override
    public double evaluate(double[] input) {
        return func.applyAsDouble(input);
    }

    @Override
    public double[] gradient(double[] input) {
        double[] inputCopy = new double[input.length];
        System.arraycopy(input, 0, inputCopy, 0, input.length);
        double[] res = new double[inputCopy.length];
        for (int i = 0; i < inputCopy.length; i++) {
            inputCopy[i] = input[i] + h;
            double fxph = evaluate(inputCopy);
            inputCopy[i] = input[i] - h;
            double fxmh = evaluate(inputCopy);
            inputCopy[i] = input[i];
            res[i] = (fxph - fxmh) / h / 2.0;
        }
        return res;
    }

}
