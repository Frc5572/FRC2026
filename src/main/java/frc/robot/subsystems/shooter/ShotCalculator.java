package frc.robot.subsystems.shooter;

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
    private static double inverseInterpolate(double a, double b, double q) {
        return (q - a) / (b - a);
    }

    private static FullShooterParams interpolate(FullShooterParams a, FullShooterParams b,
        double t) {
        return new FullShooterParams(MathUtil.interpolate(a.rps, b.rps, t),
            MathUtil.interpolate(a.hoodAngle, b.hoodAngle, t),
            MathUtil.interpolate(a.timeOfFlight, b.timeOfFlight, t));
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

    public record FullShooterParams(double rps, double hoodAngle, double timeOfFlight) {
    }


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
