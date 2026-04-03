package frc.robot.shotdata;

import org.ejml.data.DMatrix6;
import edu.wpi.first.math.geometry.Translation3d;

public class SimulatedShot {

    private static final double GRAVITY = 9.81; // m/s^2
    private static final double AIR_DENSITY = 1.225; // kg/m^3, standard atmosphere
    private static final double BALL_MASS = 0.215; // kg, game manual 5.10.1 midpoint
    private static final double BALL_DIAMETER = 0.1501; // m, game manual 5.10.1
    private static final double BALL_RADIUS = BALL_DIAMETER / 2.0;
    private static final double BALL_CROSS_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
    private static final double BALL_MOMENT_OF_INERTIA =
        0.4 * BALL_MASS * BALL_RADIUS * BALL_RADIUS; // 2/5 * m * r^2, solid sphere

    // Aerodynamic coefficients
    private static final double DEFAULT_CD = 0.47; // drag coefficient, smooth sphere
    private static final double DEFAULT_CM = 0.2; // Magnus coefficient, conservative estimate

    // Precomputed force factors (divided by mass to get acceleration factors)
    private static final double DRAG_ACCEL_FACTOR =
        0.5 * AIR_DENSITY * DEFAULT_CD * BALL_CROSS_AREA / BALL_MASS;
    // Extra BALL_RADIUS factor converts the omega x v cross product to acceleration
    private static final double MAGNUS_ACCEL_FACTOR =
        0.5 * AIR_DENSITY * DEFAULT_CM * BALL_CROSS_AREA * BALL_RADIUS / BALL_MASS;

    public double backspin;
    public DMatrix6 state = new DMatrix6();
    private DMatrix6 forces = new DMatrix6();

    public Translation3d getPosition() {
        return new Translation3d(state.a1, state.a2, state.a3);
    }

    private double horizontalVelocity() {
        return Math.hypot(state.a4, state.a5);
    }

    private void updateForces() {
        /*
         * v = [exitVelocity * cos(angle), 0, exitVelocity * sin(angle)] ω = [0, -backspin, 0]
         * magnus = ρ * π * r^3 * C_L * cross(ω, v) / 2 drag = [ρ * exitVelocity^2 * C_D * π * r^2 *
         * -(v ./ exitVelocity)...] gravity = [0, 0, -m * g] return gravity .+ drag .+ magnus
         */
    }

}
