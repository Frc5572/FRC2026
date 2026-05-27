package frc.gen;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class SimulatedShot {

    // physical constants; update for the actual game piece
    private static final double BALL_RADIUS_M = 0.12065;  // 9.5 in diameter
    private static final double BALL_MASS_KG = 0.148;
    private static final double BALL_AREA_M2 = Math.PI * BALL_RADIUS_M * BALL_RADIUS_M;

    private static final double AIR_DENSITY = 1.225;  // kg/m³ at sea level, 20 °C
    private static final double DRAG_COEFF = 0.47;    // smooth sphere
    private static final double MAGNUS_COEFF = 0.50;  // C_L; tune empirically

    private static final double G = 9.81;  // m/s²

    private double x;
    private double y;
    private double vx;
    private double vy;
    private final double omega;  // rad/s — positive = backspin (CCW from right side)

    /**
     * @param exitVelocity speed of the ball exiting the shooter
     * @param exitAngle angle of the ball's velocity, with 0 deg being straight forward and 90 deg
     *        being straight up
     * @param radialVelocity velocity component in the radial direction, with positive values meaning
     *        the robot is moving away from the target
     * @param backspin angular velocity of backspin; positive for a ball shot forward
     */
    public SimulatedShot(LinearVelocity exitVelocity, Angle exitAngle,
            LinearVelocity radialVelocity, AngularVelocity backspin) {
        double speed = exitVelocity.in(MetersPerSecond);
        double angle = exitAngle.in(Radians);

        x = 0.0;
        y = 0.0;
        vx = speed * Math.cos(angle) - radialVelocity.in(MetersPerSecond);
        vy = speed * Math.sin(angle);
        omega = backspin.in(RadiansPerSecond);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getXVelocity() { return vx; }
    public double getYVelocity() { return vy; }

    /** Advances the simulation by {@code dt} seconds using 4th-order Runge-Kutta integration. */
    public void step(double dt) {
        double[] s = {x, y, vx, vy};
        double[] k1 = deriv(s);
        double[] k2 = deriv(addScaled(s, k1, 0.5 * dt));
        double[] k3 = deriv(addScaled(s, k2, 0.5 * dt));
        double[] k4 = deriv(addScaled(s, k3, dt));

        x  = s[0] + dt / 6.0 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
        y  = s[1] + dt / 6.0 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
        vx = s[2] + dt / 6.0 * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]);
        vy = s[3] + dt / 6.0 * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]);
    }

    /**
     * Returns [dx/dt, dy/dt, dvx/dt, dvy/dt] for state s = [x, y, vx, vy].
     *
     * <p>Forces modeled:
     * <ul>
     *   <li>Gravity: (0, -mg)
     *   <li>Aerodynamic drag: -½ρ C_d A |v| · v
     *   <li>Magnus lift from backspin: ½ρ C_L A r · (ω × v)
     *       In 2D with ω on the z-axis: (-ω·vy, +ω·vx) — backspin gives upward lift.
     * </ul>
     */
    private double[] deriv(double[] s) {
        double pvx = s[2], pvy = s[3];
        double speed = Math.sqrt(pvx * pvx + pvy * pvy);

        double kDrag   = 0.5 * AIR_DENSITY * DRAG_COEFF   * BALL_AREA_M2 * speed          / BALL_MASS_KG;
        double kMagnus = 0.5 * AIR_DENSITY * MAGNUS_COEFF * BALL_AREA_M2 * BALL_RADIUS_M * omega / BALL_MASS_KG;

        double ax = -kDrag * pvx - kMagnus * pvy;
        double ay = -G - kDrag * pvy + kMagnus * pvx;

        return new double[]{pvx, pvy, ax, ay};
    }

    private static double[] addScaled(double[] a, double[] b, double s) {
        return new double[]{a[0] + b[0]*s, a[1] + b[1]*s, a[2] + b[2]*s, a[3] + b[3]*s};
    }
}
