package frc.robot.bolistics;

public class Ballistics implements DerivableFunction {

    private double gravity;
    private double density;
    private double area;
    private double mass;
    private double dragCoefficient;

    public Ballistics(double gravity, double density, double area, double mass,
        double dragCoefficient) {
        this.gravity = gravity;
        this.density = density;
        this.area = area;
        this.mass = mass;
        this.dragCoefficient = dragCoefficient;
    }

    @Override
    public double[] derivative(double[] x) {
        // TODO Auto-generated method stub
        double position = x[0];
        double velocity = x[1];

        double vmag = Math.abs(velocity);

        double f_drag = (1 / 2) * density * area * dragCoefficient * velocity * vmag;
        double f_gravity = -1 * mass * gravity;

        double f_total = f_drag + f_gravity;
        double acceleration = f_total / mass;

        return new double[] {velocity, acceleration};
    }

}
