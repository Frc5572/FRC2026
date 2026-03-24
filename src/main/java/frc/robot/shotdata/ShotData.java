package frc.robot.shotdata;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class ShotData {

    private static final ShotEntry[] entries = new ShotEntry[] {
        // @formatter:off

        // @formatter:on
    };

    private static final InterpolatingDoubleTreeMap distanceToHoodAngle =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeed =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap distanceToTimeOfFlight =
        new InterpolatingDoubleTreeMap();

    static {
        for (var entry : entries) {
            distanceToHoodAngle.put(Units.feetToMeters(entry.distanceFeet()), entry.hoodAngleDeg());
            distanceToFlywheelSpeed.put(Units.feetToMeters(entry.distanceFeet()),
                entry.flywheelSpeedRps());
            distanceToTimeOfFlight.put(Units.feetToMeters(entry.distanceFeet()),
                entry.timeOfFlight());
        }
    }

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
        double desiredSpeed = distanceToFlywheelSpeed.get(distance);
        double hood = distanceToHoodAngle.get(distance);
        double tof = distanceToTimeOfFlight.get(distance);
        double minSpeed = desiredSpeed - 10;
        boolean isOkayToShoot = currentFlywheelSpeed > minSpeed;
        if (log) {
            Logger.recordOutput("ShotParameters/desiredSpeed", desiredSpeed);
            Logger.recordOutput("ShotParameters/distance", distance);
            Logger.recordOutput("ShotParameters/currentSpeed", currentFlywheelSpeed);
            Logger.recordOutput("ShotParameters/hoodDeg", hood);
            Logger.recordOutput("ShotParameters/tof", tof);
            Logger.recordOutput("ShotParameters/isOkayToShoot", isOkayToShoot);
        }
        return new ShotParameters(desiredSpeed, hood, tof, isOkayToShoot);
    }

}
