package frc.robot.math.interp2d;

public interface MulAdd<T> {

    public T mul(T a, double b);

    public T add(T a, T b);

}
