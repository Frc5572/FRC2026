package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import frc.robot.math.interp2d.BilinearSurrogate;
import frc.robot.math.interp2d.Interp2d;
import frc.robot.math.interp2d.MulAdd;
import frc.robot.math.interp2d.RangeOf;

public class ShotData {

    // @formatter:off
    public static final ShotEntry[] entries = new ShotEntry[] {
            new ShotEntry(0.0, 0.0, 0.0, 1.0),
            new ShotEntry(1.0, 0.0, 1.0, 1.0),
            new ShotEntry(2.0, 1.0, 2.0, 1.0),
            new ShotEntry(0.0, 1.0, 3.0, 1.0),
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

    public static final record ShotEntry(double distanceMeters, double flywheelSpeedRps,
        double hoodAngleDeg, double timeOfFlight) {

        /** Vertical Exit velocity */
        public double verticalVelocity() {
            // z0 + v0 * t - 1/2 * g * t^2 = z1
            // v0 = (z1 - z0 + 1/2 * g * t^2) / t

            return (FieldConstants.Hub.innerHeight - Constants.Shooter.shooterHeight.in(Meters)
                + 0.5 * 9.81 * timeOfFlight * timeOfFlight) / timeOfFlight;
        }

        /** Horizontal Exit velocity */
        public double horizontalVelocity() {
            return distanceMeters / timeOfFlight;
        }

    }



    public static final Interp2d<ShotEntry> distanceFlywheelSpeed = new Interp2d<ShotEntry>(entries,
        mulAdd, ShotEntry::distanceMeters, ShotEntry::flywheelSpeedRps);
    public static final BilinearSurrogate<ShotEntry> distanceFlywheelSpeedSurrogate =
        distanceFlywheelSpeed.surrogate(new RangeOf().min(0).max(30.0).discretization(60).finish(),
            new RangeOf().min(45.0).max(100.0).discretization(30).finish());

    public static final Interp2d<ShotEntry> flywheelSpeedHoodAngle = new Interp2d<ShotEntry>(
        entries, mulAdd, ShotEntry::flywheelSpeedRps, ShotEntry::hoodAngleDeg);
    public static final BilinearSurrogate<ShotEntry> flywheelSpeedHoodAngleSurrogate =
        flywheelSpeedHoodAngle.surrogate(
            new RangeOf().min(45.0).max(100.0).discretization(30).finish(),
            new RangeOf().min(0).max(45.0).discretization(30).finish());

}
