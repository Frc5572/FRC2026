package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.math.interp2d.Interp2d;
import frc.robot.math.interp2d.MulAdd;

/** Data for flywheel, distance, and hood angle that results in a successful shot. */
public class ShotData {

    private static final InterpolatingDoubleTreeMap minFlywheelSpeed =
        new InterpolatingDoubleTreeMap();

    /** Raw data from testing. */
    // @formatter:off
    public static final ShotEntry[] entries = new ShotEntry[] {
        new ShotEntry(6.0, 60.0, 6.0, 1.63),
        new ShotEntry(6.0, 50.0, 10.0, 1.15),
        new ShotEntry(6.0, 45.0, 10.0, 1.12),

        new ShotEntry(8.0, 60.0, 8.0, 1.56),
        new ShotEntry(8.0, 50.0, 15.0, 1.28),

        new ShotEntry(10.0, 60.0, 10.0, 1.58),
        new ShotEntry(10.0, 50.0, 20.0, 1.19),
        new ShotEntry(10.0, 70.0, 5.0, 2.08),

        new ShotEntry(12.0, 60.0, 12.0, 1.56),
        new ShotEntry(12.0, 55.0, 15.0, 1.38),
        new ShotEntry(12.0, 70.0, 7.0, 2.0),

        new ShotEntry(14.0, 60.0, 16.0, 1.40),
        // new ShotEntry(14.0, 55.0, 20.0, 1.18),
        new ShotEntry(14.0, 70.0, 11.2, 1.72),

        new ShotEntry(16.0, 60.0, 20.0, 1.48),
        new ShotEntry(16.0, 70.0, 17.0, 1.72),

        new ShotEntry(18.0, 70.0, 26.0, 1.54),
    };
    // @formatter:on

    static {
        HashMap<Double, Double> minFlywheelSpeedMap = new HashMap<>();
        for (ShotEntry entry : entries) {
            Double v = minFlywheelSpeedMap.get(entry.distanceFeet);
            if (v == null || v > entry.flywheelSpeedRps) {
                minFlywheelSpeedMap.put(entry.distanceFeet, entry.flywheelSpeedRps);
            }
        }
        for (var entry : minFlywheelSpeedMap.entrySet()) {
            minFlywheelSpeed.put(entry.getKey(), entry.getValue());
        }
    }

    private static final double MIN_DESIRED_SPEED = 50.0;
    private static final double MIN_DESIRED_SPEED_DISTANCE = 6.0;

    private static final double MAX_DESIRED_SPEED = 70.0;
    private static final double MAX_DESIRED_SPEED_DISTANCE = 18.0;

    private static final double SPEED_M = (MAX_DESIRED_SPEED - MIN_DESIRED_SPEED)
        / (MAX_DESIRED_SPEED_DISTANCE - MIN_DESIRED_SPEED_DISTANCE);
    private static final double SPEED_B = MAX_DESIRED_SPEED - SPEED_M * MAX_DESIRED_SPEED_DISTANCE;

    public static final MulAdd<ShotEntry> mulAdd = new MulAdd<ShotData.ShotEntry>() {

        @Override
        public ShotEntry mul(ShotEntry a, double b) {
            return new ShotEntry(a.distanceFeet * b, a.flywheelSpeedRps * b, a.hoodAngleDeg * b,
                a.timeOfFlight * b);
        }

        @Override
        public ShotEntry add(ShotEntry a, ShotEntry b) {
            return new ShotEntry(a.distanceFeet + b.distanceFeet,
                a.flywheelSpeedRps + b.flywheelSpeedRps, a.hoodAngleDeg + b.hoodAngleDeg,
                a.timeOfFlight + b.timeOfFlight);
        }

    };

    /** Entry into the shot data table */
    public static final record ShotEntry(double distanceFeet, double flywheelSpeedRps,
        double hoodAngleDeg, double timeOfFlight) {

        /** Vertical exit velocity */
        public double verticalVelocity() {
            // z0 + v0 * t - 1/2 * g * t^2 = z1
            // v0 = (z1 - z0 + 1/2 * g * t^2) / t

            return (FieldConstants.Hub.innerHeight - Constants.Shooter.shooterHeight.in(Meters)
                + 0.5 * 9.81 * timeOfFlight * timeOfFlight) / timeOfFlight;
        }

        /** Horizontal exit velocity */
        public double horizontalVelocity() {
            return Units.feetToMeters(distanceFeet) / timeOfFlight;
        }

    }

    public static Interp2d<ShotEntry> distanceFlywheel =
        new Interp2d<>(entries, mulAdd, ShotEntry::distanceFeet, ShotEntry::flywheelSpeedRps);
    public static Interp2d<ShotEntry> flywheelHood =
        new Interp2d<>(entries, mulAdd, ShotEntry::flywheelSpeedRps, ShotEntry::hoodAngleDeg);

    /** Parameters for a single instance of shooting. */
    public static record ShotParameters(double desiredSpeed, double hoodAngleDeg,
        double timeOfFlight, boolean isOkayToShoot) {
    }

    /** Get parameters for a given shot situation. */
    public static ShotParameters getShotParameters(double distance, double currentFlywheelSpeed,
        boolean log) {
        double desiredSpeed =
            MathUtil.clamp(SPEED_M * distance + SPEED_B, MIN_DESIRED_SPEED, MAX_DESIRED_SPEED);
        var q = distanceFlywheel.query(new Translation2d(distance, currentFlywheelSpeed));
        double hood = q.value().hoodAngleDeg();
        double tof = q.value().timeOfFlight();
        double minSpeed = minFlywheelSpeed.get(distance);
        boolean isOkayToShoot = q.sdf() < 0.02 || currentFlywheelSpeed > minSpeed;
        if (log) {
            Logger.recordOutput("ShotParameters/desiredSpeed", desiredSpeed);
            Logger.recordOutput("ShotParameters/distance", distance);
            Logger.recordOutput("ShotParameters/currentSpeed", currentFlywheelSpeed);
            Logger.recordOutput("ShotParameters/hoodDeg", hood);
            Logger.recordOutput("ShotParameters/tof", tof);
            Logger.recordOutput("ShotParameters/sdf", q.sdf());
            Logger.recordOutput("ShotParameters/isOkayToShoot", isOkayToShoot);
        }
        return new ShotParameters(desiredSpeed, hood, tof, isOkayToShoot);
    }

}
