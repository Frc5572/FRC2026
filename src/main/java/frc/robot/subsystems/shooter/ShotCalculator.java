package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class ShotCalculator {
    private static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP =
        new InterpolatingTreeMap<Double, FullShooterParams>((a, b, q) -> 0.0,
            (a, b, t) -> new FullShooterParams(0, 0, 0));
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

    public record FullShooterParams(double rpm, double hoodAngle, double timeOfFlight) {
    }
}
