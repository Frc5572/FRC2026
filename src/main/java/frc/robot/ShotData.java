package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import frc.robot.math.interp2d.Interp2d;
import frc.robot.math.interp2d.MulAdd;

/** Data for flywheel, distance, and hood angle that results in a successful shot. */
public class ShotData {

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
        new ShotEntry(14.0, 55.0, 20.0, 1.18),
        new ShotEntry(14.0, 70.0, 11.2, 1.72),

        new ShotEntry(16.0, 60.0, 20.0, 1.48),
        new ShotEntry(16.0, 70.0, 17.0, 1.72),

        new ShotEntry(18.0, 70.0, 26.0, 1.54),
    };
    // @formatter:on

    private static final MulAdd<ShotEntry> mulAdd = new MulAdd<ShotData.ShotEntry>() {

        @Override
        public ShotEntry mul(ShotEntry a, double b) {
            return new ShotEntry(a.distanceMeters * b, a.flywheelSpeedRps * b, a.hoodAngleDeg * b,
                a.timeOfFlight * b);
        }

        @Override
        public ShotEntry add(ShotEntry a, ShotEntry b) {
            return new ShotEntry(a.distanceMeters + b.distanceMeters,
                a.flywheelSpeedRps + b.flywheelSpeedRps, a.hoodAngleDeg + b.hoodAngleDeg,
                a.timeOfFlight + b.timeOfFlight);
        }

    };

    /** Entry into the shot data table */
    public static final record ShotEntry(double distanceMeters, double flywheelSpeedRps,
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
            return distanceMeters / timeOfFlight;
        }

    }


    public static final Interp2d<ShotEntry> distanceFlywheelSpeed = new Interp2d<ShotEntry>(entries,
        mulAdd, ShotEntry::distanceMeters, ShotEntry::flywheelSpeedRps);

    public static final Interp2d<ShotEntry> flywheelSpeedHoodAngle = new Interp2d<ShotEntry>(
        entries, mulAdd, ShotEntry::flywheelSpeedRps, ShotEntry::hoodAngleDeg);

}
