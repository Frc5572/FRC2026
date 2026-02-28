package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import frc.robot.math.interp2d.BilinearSurrogate;
import frc.robot.math.interp2d.Interp2d;
import frc.robot.math.interp2d.MulAdd;
import frc.robot.math.interp2d.Range;
import frc.robot.math.interp2d.RangeOf;

public class ShotData {

    // @formatter:off
    public static final ShotEntry[] entries = new ShotEntry[] {
        new ShotEntry(6.0, 60.0, 6.0, 1.63),
        new ShotEntry(6.0, 50.0, 10.0, 1.15),
        new ShotEntry(6.0, 45.0, 10.0, 1.12),
        
        new ShotEntry(8.0, 60.0, 8.0, 1.56),
        new ShotEntry(8.0, 50.0, 15.0, 1.28),
        new ShotEntry(10.0, 70.0, 0.0, 2.09),
        
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
        // new ShotEntry(0.0, 0.0, 0.0, 1.0),
        //     new ShotEntry(1.0, 0.0, 1.0, 1.0),
        //     new ShotEntry(2.0, 1.0, 2.0, 1.0),
        //     new ShotEntry(0.0, 1.0, 3.0, 1.0),
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

    public static final Range distanceRange =
        new RangeOf().min(0.0).max(22.0).discretization(20).finish();
    public static final Range flywheelRange =
        new RangeOf().min(40.0).max(80.0).discretization(20).finish();
    public static final Range hoodRange =
        new RangeOf().min(0.0).max(30.0).discretization(30).finish();

    public static final Interp2d<ShotEntry> distanceFlywheelSpeed = new Interp2d<ShotEntry>(entries,
        mulAdd, ShotEntry::distanceMeters, ShotEntry::flywheelSpeedRps);
    public static final BilinearSurrogate<ShotEntry> distanceFlywheelSpeedSurrogate =
        distanceFlywheelSpeed.surrogate(distanceRange, flywheelRange);

    public static final Interp2d<ShotEntry> flywheelSpeedHoodAngle = new Interp2d<ShotEntry>(
        entries, mulAdd, ShotEntry::flywheelSpeedRps, ShotEntry::hoodAngleDeg);
    public static final BilinearSurrogate<ShotEntry> flywheelSpeedHoodAngleSurrogate =
        flywheelSpeedHoodAngle.surrogate(flywheelRange, hoodRange);

}
