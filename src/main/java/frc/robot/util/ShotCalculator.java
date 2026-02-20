package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
     * @param hoodAngle The required hood angle in degrees to achieve the desired trajectory at the
     *        given distance.
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

    /**
     * Calculates shooter parameters accounting for robot motion during projectile flight time using
     * iterative lookahead compensation. Accounts for the fact that the robot will move during the
     * projectile's flight, so the required distance and velocity are recalculated iteratively until
     * convergence.
     * 
     * @param launcherPosition The current position of the launcher on the field as a Translation2d.
     * @param chassisSpeeds The current velocity of the robot as a ChassisSpeeds object (vx, vy,
     *        omega).
     * @param targetPosition The position of the target on the field as a Translation2d.
     * @param rpsOutput A consumer to accept the calculated shooter RPS.
     * @param hoodAngle A consumer to accept the calculated hood angle.
     * @return The distance to the target after lookahead compensation.
     */
    public static double velocityComp(Translation2d launcherPosition, ChassisSpeeds chassisSpeeds,
        Translation2d targetPosition, Consumer<Double> rpsOutput, Consumer<Angle> hoodAngle) {

        // Convert chassis speeds to velocity vector
        Translation2d launcherVelocity =
            new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        // Initial distance calculation
        double distance = targetPosition.getDistance(launcherPosition);

        // Iterative lookahead: account for launcher movement during projectile flight
        double lookaheadDistance = distance;
        double timeOfFlight;

        for (int i = 0; i < 20; i++) {
            // Get time of flight for current distance
            ShooterParams baseline = SHOOTER_MAP.get(lookaheadDistance);
            timeOfFlight = baseline.timeOfFlight;

            // Calculate where launcher will be after projectile flight time
            double offsetX = launcherVelocity.getX() * timeOfFlight;
            double offsetY = launcherVelocity.getY() * timeOfFlight;
            Translation2d lookaheadLauncherPos =
                launcherPosition.plus(new Translation2d(offsetX, offsetY));

            // Recalculate distance from lookahead position to target
            lookaheadDistance = targetPosition.getDistance(lookaheadLauncherPos);
        }

        // Calculate final shooter parameters
        ShooterParams baseline = SHOOTER_MAP.get(lookaheadDistance);
        double baselineVelocity = lookaheadDistance / baseline.timeOfFlight;

        // Calculate target direction
        Translation2d toTarget = targetPosition.minus(launcherPosition);
        Translation2d targetDirection = toTarget.div(toTarget.getNorm());

        // Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // Subtract launcher velocity to get required shot velocity
        Translation2d shotVelocity = targetVelocity.minus(launcherVelocity);
        double requiredVelocity = shotVelocity.getNorm();

        calculateBoth(lookaheadDistance, requiredVelocity, hoodAngle, rpsOutput);

        return lookaheadDistance;
    }

    /**
     * Retrieves shooter parameters with iterative lookahead velocity compensation applied. Accounts
     * for the robot moving during projectile flight time by iteratively calculating the launcher's
     * future position.
     * 
     * @param launcherPosition The current position of the launcher on the field as a Translation2d.
     * @param chassisSpeeds The current velocity of the robot as a ChassisSpeeds object (vx, vy,
     *        omega).
     * @param targetPosition The position of the target on the field as a Translation2d.
     * @return A ShooterParams object containing the RPS and hood angle adjusted for velocity
     *         compensation, along with the lookahead distance and time of flight.
     */
    public static ShooterParams velocityCompParams(Translation2d launcherPosition,
        ChassisSpeeds chassisSpeeds, Translation2d targetPosition) {
        Translation2d launcherVelocity =
            new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        double distance = targetPosition.getDistance(launcherPosition);
        double lookaheadDistance = distance;
        double timeOfFlight;

        for (int i = 0; i < 20; i++) {
            ShooterParams baseline = SHOOTER_MAP.get(lookaheadDistance);
            timeOfFlight = baseline.timeOfFlight;
            double offsetX = launcherVelocity.getX() * timeOfFlight;
            double offsetY = launcherVelocity.getY() * timeOfFlight;
            Translation2d lookaheadLauncherPos =
                launcherPosition.plus(new Translation2d(offsetX, offsetY));
            lookaheadDistance = targetPosition.getDistance(lookaheadLauncherPos);
        }

        ShooterParams baseline = SHOOTER_MAP.get(lookaheadDistance);
        double baselineVelocity = lookaheadDistance / baseline.timeOfFlight;
        Translation2d toTarget = targetPosition.minus(launcherPosition);
        Translation2d targetDirection = toTarget.div(toTarget.getNorm());
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);
        Translation2d shotVelocity = targetVelocity.minus(launcherVelocity);
        double requiredVelocity = shotVelocity.getNorm();
        double velocityRatio = requiredVelocity / baselineVelocity;
        double rpsFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);
        double adjustedRps = baseline.rps * rpsFactor;
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle));
        double targetHorizFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
        double adjustedHoodAngle = Math.toDegrees(Math.acos(ratio));

        return new ShooterParams(adjustedRps, adjustedHoodAngle, baseline.timeOfFlight);
    }

    /**
     * Retrieves baseline shooter parameters for a given distance without any velocity compensation.
     * 
     * @param distance A DoubleSupplier that provides the distance in meters.
     * @return The ShooterParams from the map at the specified distance.
     */
    public static ShooterParams staticShotparams(DoubleSupplier distance) {
        return SHOOTER_MAP.get(distance.getAsDouble());
    }
}
