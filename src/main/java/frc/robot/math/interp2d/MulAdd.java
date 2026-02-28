package frc.robot.math.interp2d;

/** Utilities for multiplying and adding */
public interface MulAdd<T> {

    /** multiply by scalar */
    public T mul(T a, double b);

    /** add two data */
    public T add(T a, T b);

}
