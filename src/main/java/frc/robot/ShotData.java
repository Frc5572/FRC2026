package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Data for flywheel, distance, and hood angle that results in a successful shot. */
public class ShotData {

    public static final ShotEntry[] entries = new ShotEntry[] {};

    public static final InterpolatingDoubleTreeMap distanceToFlywheel =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToHoodAngle =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToTimeOfFlight =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap hoodAngleToExitAngle =
        new InterpolatingDoubleTreeMap();

    static {
        for (ShotEntry entry : entries) {
            distanceToFlywheel.put(entry.distanceFeet(), entry.flywheelSpeedRps());
            distanceToHoodAngle.put(entry.distanceFeet(), entry.hoodAngleDeg());
            distanceToTimeOfFlight.put(entry.distanceFeet(), entry.timeOfFlight());
            hoodAngleToExitAngle.put(Units.degreesToRadians(entry.hoodAngleDeg()),
                entry.exitAngle());
            hoodAngleToExitAngle.put(Units.degreesToRadians(entry.hoodAngleDeg()),
                entry.exitAngle());
        }
    }

    /** Entry into the shot data table */
    public static final record ShotEntry(double distanceFeet, double flywheelSpeedRps,
        double hoodAngleDeg, double timeOfFlight) {

        /** Vertical exit velocity in m/s */
        public double verticalVelocity() {
            // z0 + v0 * t - 1/2 * g * t^2 = z1
            // v0 = (z1 - z0 + 1/2 * g * t^2) / t

            return (FieldConstants.Hub.innerHeight - Constants.Shooter.shooterHeight.in(Meters)
                + 0.5 * 9.81 * timeOfFlight * timeOfFlight) / timeOfFlight;
        }

        /** Horizontal exit velocity in m/s */
        public double horizontalVelocity() {
            return Units.feetToMeters(distanceFeet) / timeOfFlight;
        }

        /** Exit angle in radians */
        public double exitAngle() {
            return Math.atan2(verticalVelocity(), horizontalVelocity());
        }

        /** Exit speed in m/s */
        public double exitSpeed() {
            return Math.hypot(verticalVelocity(), horizontalVelocity());
        }

    }

    /** Parameters for a single instance of shooting. */
    public static record ShotParameters(double desiredSpeed, double hoodAngleDeg,
        double timeOfFlight, boolean isOkayToShoot) {
    }

    /** Get parameters for a given shot situation. */
    public static ShotParameters getShotParameters(double distance, double currentFlywheelSpeed,
        boolean log) {
        // TODO
        return new ShotParameters(0.0, 0.0, 0.0, true);
    }

}
