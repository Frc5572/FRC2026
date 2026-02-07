package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.Consumer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;

/**
 * Calculates shooter parameters (RPM and hood angle) based on distance and required velocity. Uses
 * an interpolating map of empirically derived shooter parameters at various distances, then applies
 * a correction based on the required velocity. The correction is split between RPM and hood angle
 * to allow for more nuanced adjustments.
 */
public class ShotCalculator {

    /**
     * Inverse interpolation function to calculate the interpolation factor t based on the query
     * value and the start/end values. This is used by the InterpolatingTreeMap to determine how far
     * along the query is between the two bounding keys.
     * 
     * @param startValue The value at the lower bound key.
     * @param endValue The value at the upper bound key.
     * @param query The value for which we want to find the interpolation factor.
     * @return The interpolation factor t, where 0 corresponds to startValue and 1 corresponds to
     *         endValue.
     */
    private static double inverseInterpolate(double startValue, double endValue, double query) {
        return (query - startValue) / (endValue - startValue);
    }

    /**
     * Per-parameter interpolation for shooter parameters. Interpolates each parameter independently
     * based on the interpolation factor t.
     * 
     * @param startValue The shooter parameters at the lower bound distance.
     * @param endValue The shooter parameters at the upper bound distance.
     * @param t The interpolation factor between 0 and 1, where 0 corresponds to startValue and 1
     *        corresponds to endValue.
     * @return A new FullShooterParams object with each parameter interpolated based on t.
     */
    private static FullShooterParams interpolate(FullShooterParams startValue,
        FullShooterParams endValue, double t) {
        return new FullShooterParams(MathUtil.interpolate(startValue.rps, endValue.rps, t),
            MathUtil.interpolate(startValue.hoodAngle, endValue.hoodAngle, t),
            MathUtil.interpolate(startValue.timeOfFlight, endValue.timeOfFlight, t));
    }

    private static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP =
        new InterpolatingTreeMap<Double, FullShooterParams>(ShotCalculator::inverseInterpolate,
            ShotCalculator::interpolate);

    static {
        SHOOTER_MAP.put(1.5, new FullShooterParams(2800.0, 35.0, 0.38));
        SHOOTER_MAP.put(2.0, new FullShooterParams(3100.0, 38.0, 0.45));
        SHOOTER_MAP.put(2.5, new FullShooterParams(3400.0, 42.0, 0.52));
        SHOOTER_MAP.put(3.0, new FullShooterParams(3650.0, 46.0, 0.60));
        SHOOTER_MAP.put(3.5, new FullShooterParams(3900.0, 50.0, 0.68));
        SHOOTER_MAP.put(4.0, new FullShooterParams(4100.0, 54.0, 0.76));
        SHOOTER_MAP.put(4.5, new FullShooterParams(4350.0, 58.0, 0.85));
        SHOOTER_MAP.put(5.0, new FullShooterParams(4550.0, 62.0, 0.94));
    }

    /**
     * Parameters for a shooter shot, including RPS, hood angle, and time of flight.
     * 
     * @param rps The required shooter RPS to achieve the desired velocity at the given distance.
     * @param hoodAngle The required hood angle to achieve the desired trajectory at the given
     *        distance.
     * @param timeOfFlight The expected time of flight for a shot at the given distance with the
     *        baseline parameters. Used for velocity correction calculations.
     */
    public record FullShooterParams(double rps, double hoodAngle, double timeOfFlight) {
    }

    /**
     * Calculates shooter parameters based on distance and required velocity.
     * 
     * @param distance The distance to the target in meters.
     * @param requiredVelocity The required velocity of the projectile at the target in m/s.
     * @param hoodAngle A consumer to accept the calculated hood angle.
     * @param rpsOutput A consumer to accept the calculated shooter RPS.
     * 
     * 
     */
    public static void calculateBoth(double distance, double requiredVelocity,
        Consumer<Angle> hoodAngle, Consumer<Double> rpsOutput) {
        FullShooterParams baseline = SHOOTER_MAP.get(distance);
        double baselineVelocity = distance / baseline.timeOfFlight;
        double velocityRatio = requiredVelocity / baselineVelocity;

        // Split the correction: sqrt gives equal "contribution" from each
        double rpsFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);

        // Apply RPM scaling
        double adjustedRps = baseline.rps * rpsFactor;

        // Apply hood adjustment (changes horizontal component)
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle));
        double targetHorizFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
        Angle adjustedHood = Radians.of(Math.acos(ratio));

        hoodAngle.accept(adjustedHood);
        rpsOutput.accept(adjustedRps);
    }
}
