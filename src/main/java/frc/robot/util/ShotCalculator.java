package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import java.util.Map;
import java.util.function.Consumer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
     * @return A new ShooterParams object with each parameter interpolated based on t.
     */
    private static ShooterParams interpolate(ShooterParams startValue, ShooterParams endValue,
        double t) {
        return new ShooterParams(MathUtil.interpolate(startValue.rps, endValue.rps, t),
            MathUtil.interpolate(startValue.hoodAngle, endValue.hoodAngle, t),
            MathUtil.interpolate(startValue.timeOfFlight, endValue.timeOfFlight, t));
    }

    private static final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_MAP =
        new InterpolatingTreeMap<Double, ShooterParams>(ShotCalculator::inverseInterpolate,
            ShotCalculator::interpolate);

    static {
        SHOOTER_MAP.put(1.5, new ShooterParams(2800.0, 35.0, 0.38));
        SHOOTER_MAP.put(2.0, new ShooterParams(3100.0, 38.0, 0.45));
        SHOOTER_MAP.put(2.5, new ShooterParams(3400.0, 42.0, 0.52));
        SHOOTER_MAP.put(3.0, new ShooterParams(3650.0, 46.0, 0.60));
        SHOOTER_MAP.put(3.5, new ShooterParams(3900.0, 50.0, 0.68));
        SHOOTER_MAP.put(4.0, new ShooterParams(4100.0, 54.0, 0.76));
        SHOOTER_MAP.put(4.5, new ShooterParams(4350.0, 58.0, 0.85));
        SHOOTER_MAP.put(5.0, new ShooterParams(4550.0, 62.0, 0.94));
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
    public record ShooterParams(double rps, double hoodAngle, double timeOfFlight) {
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
        ShooterParams baseline = SHOOTER_MAP.get(distance);
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

    public static void velocityComp(Translation2d robotPosition, Translation2d robotVelocity,
        Translation2d goalPosition, Consumer<Double> rpsOutput, Consumer<Angle> hoodAngle) {

        // 1. Project future position
        Translation2d futurePos = robotPosition.plus(robotVelocity);

        // 2. Get target vector
        Translation2d toGoal = goalPosition.minus(futurePos);
        double distance = toGoal.getNorm();
        Translation2d targetDirection = toGoal.div(distance);

        // 3. Look up baseline velocity from table
        ShooterParams baseline = SHOOTER_MAP.get(distance);
        double baselineVelocity = distance / baseline.timeOfFlight;

        // 4. Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 5. THE MAGIC: subtract robot velocity
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

        // 6. Extract results
        Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        // 7. Use table in reverse: velocity → effective distance → RPM
        double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
        double requiredRpm = SHOOTER_MAP.get(effectiveDistance).rps;
    }

    public double velocityToEffectiveDistance(double velocity) {
        // Binary search or iterate through table to find distance
        // where (distance / ToF) = velocity
        // Most InterpolatingTreeMap implementations support inverse lookup
        // or you can build a reverse map: velocity → distance

        for (Map.Entry<Double, ShooterParams> entry : SHOOTER_MAP.entrySet()) {
            double dist = entry.getKey();
            double vel = dist / entry.getValue().timeOfFlight;
            if (vel >= velocity) {
                return dist; // Interpolate for better accuracy
            }
        }
        return SHOOTER_MAP.lastKey(); // Clamp to max
    }
}
