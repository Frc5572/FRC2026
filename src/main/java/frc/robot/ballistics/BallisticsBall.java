package frc.robot.ballistics;

public class BallisticsBall implements DerivableFunction {
    private double density;
    private double dragCoeffient;
    private double mass;
    private double area;
    private double gravity;

    public BallisticsBall(double density, double dragCoeffient, double mass, double area,
        double gravity) {
        this.density = density;
        this.dragCoeffient = dragCoeffient;
        this.mass = mass;
        this.area = area;
        this.gravity = gravity;
    }

    public double[] derivative(double[] x) {
        // double pos = x[0];
        double velocity = x[1];
        double vMag = Math.abs(velocity); // Math of both velocities
        double dragForce = -0.5 * dragCoeffient * density * area * vMag;
        double gravityForce = mass * gravity;
        double totalForce = dragForce + gravityForce;
        double acceleration = totalForce / mass;

        return new double[] {velocity, acceleration};
    }
}
