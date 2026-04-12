package frc.robot.math.opt;

/** Differentiable function */
public interface DiffFunc {

    /** Evaluate the function at an input */
    public double evaluate(double[] input);

    /** Get the functions gradient at an input */
    public double[] gradient(double[] input);

}
