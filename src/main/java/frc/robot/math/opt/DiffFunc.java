package frc.robot.math.opt;

public interface DiffFunc {

    public double evaluate(double[] input);

    public double[] gradient(double[] input);

}
