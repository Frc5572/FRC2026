package frc.robot.shotdata;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.math.interp2d.MulAdd;

/** Storage and interpolation for shooter parameters. */
public class ShotData {

    /** Manually acquired shot data. TOF does not need to be accurate. */
    public static ShotEntry[] entries = new ShotEntry[] {
        // @formatter:off
        new ShotEntry(14.37, 58, 22, 0),
        new ShotEntry(12.25, 55, 15, 0),
        new ShotEntry(9.25, 47, 15, 0),
        new ShotEntry(5.25, 45, 2, 0),
        new ShotEntry(7.38, 47, 7, 0),
        // @formatter:on
    };

    public static final Distance shooterToTargetHeightDiff =
        FieldConstants.Hub.topCenterPoint.getMeasureZ().minus(Constants.Shooter.shooterHeight);

    public static record ShotEntry(Distance targetDistance, AngularVelocity flywheelSpeed,
        Angle exitAngle, Time tof) {
        public ShotEntry(double distanceFeet, double flywheelSpeed, double hoodAngleDeg,
            double tof) {
            this(Feet.of(distanceFeet), RotationsPerSecond.of(flywheelSpeed),
                Degrees.of(90 - 12.695 - hoodAngleDeg), Seconds.of(tof));
        }

        public LinearVelocity theoreticalExitVelocity() {
            return MetersPerSecond.of(Math.sqrt(Math.pow(targetDistance.in(Meters), 2.0) * 9.81
                / (targetDistance.in(Meters) * Math.sin(exitAngle.in(Radians) * 2.0)
                    - 2.0 * ShotData.shooterToTargetHeightDiff.in(Meters)
                        * Math.pow(Math.cos(exitAngle.in(Radians)), 2.0))));
        }

        public LinearVelocity backtracedExitVelocity() {
            return MetersPerSecond.of(
                targetDistance.in(Meters) / Math.cos(exitAngle.in(Radians)) / tof().in(Seconds));
        }

        public Time theoreticalTimeOfFlight() {
            return Seconds.of(targetDistance.in(Meters) / Math.cos(exitAngle.in(Radians))
                / theoreticalExitVelocity().in(MetersPerSecond));
        }

        public LinearVelocity noSlipExitVelocity() {
            return MetersPerSecond.of(flywheelSpeed.in(RadiansPerSecond) * Inches.of(2).in(Meters));
        }

        public Angle hoodAngle() {
            return Degrees.of(90 - 12.695 - exitAngle.in(Degrees));
        }
    }

    public static final MulAdd<ShotEntry> mulAdd = new MulAdd<ShotEntry>() {

        @Override
        public ShotEntry mul(ShotEntry a, double b) {
            return new ShotEntry(a.targetDistance.times(b), a.flywheelSpeed.times(b),
                a.exitAngle.times(b), a.tof.times(b));
        }

        @Override
        public ShotEntry add(ShotEntry a, ShotEntry b) {
            return new ShotEntry(a.targetDistance.plus(b.targetDistance),
                a.flywheelSpeed.plus(b.flywheelSpeed), a.exitAngle.plus(b.exitAngle),
                a.tof.plus(b.tof));
        }

    };

    public static final SemiGriddedBilinearInterpolation<ShotEntry> shootFunc =
        new SemiGriddedBilinearInterpolation<>(2.0, GeneratedLUTs.hubEntries, mulAdd,
            x -> x.flywheelSpeed().in(RotationsPerSecond), x -> x.targetDistance().in(Meters));
    public static final SemiGriddedBilinearInterpolation<ShotEntry> passFunc =
        new SemiGriddedBilinearInterpolation<>(2.0, GeneratedLUTs.groundEntries, mulAdd,
            x -> x.flywheelSpeed().in(RotationsPerSecond), x -> x.targetDistance().in(Meters));
    public static final SemiGriddedBilinearInterpolation<ShotEntry> simFunc =
        new SemiGriddedBilinearInterpolation<>(2.0, GeneratedLUTs.groundEntries, mulAdd,
            x -> x.flywheelSpeed().in(RotationsPerSecond), x -> x.targetDistance().in(Meters));

    /** Parameters for a single instance of shooting. */
    public static record ShotParameters(double desiredSpeed, double hoodAngleDeg,
        double timeOfFlight, boolean isOkayToShoot) {
    }

    /** Get parameters for a given shot situation. */
    public static ShotParameters getShotParameters(double distance, double currentFlywheelSpeed,
        boolean log) {
        var res = shootFunc.interpolate(currentFlywheelSpeed, distance);
        double desiredSpeed = GeneratedLUTs.desiredFlywheelSpeed(distance);
        double hoodAngleDeg = res.hoodAngle().in(Degrees);
        double tof = res.tof().in(Seconds);
        boolean isOkay = currentFlywheelSpeed > desiredSpeed - 10;
        if (log) {
            Logger.recordOutput("ShotParameters/distance", distance);
            Logger.recordOutput("ShotParameters/currentSpeed", currentFlywheelSpeed);
            Logger.recordOutput("ShotParameters/desiredSpeed", desiredSpeed);
            Logger.recordOutput("ShotParameters/hoodAngleDeg", hoodAngleDeg);
            Logger.recordOutput("ShotParameters/tof", tof);
            Logger.recordOutput("ShotParameters/isOkay", isOkay);
        }
        return new ShotParameters(desiredSpeed, hoodAngleDeg, tof, isOkay);
    }

    /** Get parameters for a given shot situation. */
    public static ShotParameters getPassParameters(double distance, double currentFlywheelSpeed,
        boolean log) {
        var res = passFunc.interpolate(currentFlywheelSpeed, distance);
        double desiredSpeed = GeneratedLUTs.desiredFlywheelSpeed(distance);
        double hoodAngleDeg = res.hoodAngle().in(Degrees);
        double tof = res.tof().in(Seconds);
        boolean isOkay = currentFlywheelSpeed > desiredSpeed - 10;
        if (log) {
            Logger.recordOutput("ShotParameters/distance", distance);
            Logger.recordOutput("ShotParameters/currentSpeed", currentFlywheelSpeed);
            Logger.recordOutput("ShotParameters/desiredSpeed", desiredSpeed);
            Logger.recordOutput("ShotParameters/hoodAngleDeg", hoodAngleDeg);
            Logger.recordOutput("ShotParameters/tof", tof);
            Logger.recordOutput("ShotParameters/isOkay", isOkay);
        }
        return new ShotParameters(desiredSpeed, hoodAngleDeg, tof, isOkay);
    }

}
