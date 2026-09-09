package frc.robot.ballistics;

@FunctionalInterface
public interface DerivativeFunction {

    public double[] derivative(double[] input);

}
