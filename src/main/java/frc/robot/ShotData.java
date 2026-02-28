package frc.robot;

import frc.robot.math.interp2d.BilinearSurrogate;
import frc.robot.math.interp2d.Interp2d;
import frc.robot.math.interp2d.MulAdd;
import frc.robot.math.interp2d.RangeOf;

public class ShotData {

    // @formatter:off
    public static final ShotEntry[] entries = new ShotEntry[] {
            new ShotEntry(0.0, 0.0, 0.0, 0.0),
            new ShotEntry(1.0, 0.0, 1.0, 0.0),
            new ShotEntry(2.0, 1.0, 2.0, 0.0),
            new ShotEntry(0.0, 1.0, 3.0, 0.0),
    };
    // @formatter:on

    private static final MulAdd<ShotEntry> mulAdd = new MulAdd<ShotData.ShotEntry>() {

        @Override
        public ShotEntry mul(ShotEntry a, double b) {
            return new ShotEntry(a.distance * b, a.flywheelSpeedRps * b, a.hoodAngle * b,
                a.timeOfFlight * b);
        }

        @Override
        public ShotEntry add(ShotEntry a, ShotEntry b) {
            return new ShotEntry(a.distance + b.distance, a.flywheelSpeedRps + b.flywheelSpeedRps,
                a.hoodAngle + b.hoodAngle, a.timeOfFlight + b.timeOfFlight);
        }

    };

    public static final record ShotEntry(double distance, double flywheelSpeedRps, double hoodAngle,
        double timeOfFlight) {
    }



    public static final Interp2d<ShotEntry> distanceFlywheelSpeed =
        new Interp2d<ShotEntry>(entries, mulAdd, ShotEntry::distance, ShotEntry::flywheelSpeedRps);
    public static final BilinearSurrogate<ShotEntry> distanceFlywheelSpeedSurrogate =
        distanceFlywheelSpeed.surrogate(new RangeOf().min(0).max(30.0).discretization(60).finish(),
            new RangeOf().min(45.0).max(100.0).discretization(30).finish());

    // public static final BilinearSurrogate<ShotEntry> flywheelSpeedHoodAngle =
    // new Interp2d<ShotEntry>(entries, mulAdd, ShotEntry::flywheelSpeedRps, ShotEntry::hoodAngle)
    // .surrogate(20, 20);

}
