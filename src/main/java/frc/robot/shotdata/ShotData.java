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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/** Storage and interpolation for shooter parameters. */
public class ShotData {

    public static ShotEntry[] entries = new ShotEntry[] {
        // @formatter:off
        new ShotEntry(14.37, 58, 22),
        new ShotEntry(12.25, 55, 15),
        new ShotEntry(9.25, 47, 15),
        new ShotEntry(5.25, 45, 2),
        new ShotEntry(7.38, 47, 7),
        // @formatter:on
    };

    public static final Distance shooterToTargetHeightDiff =
        FieldConstants.Hub.topCenterPoint.getMeasureZ().minus(Constants.Shooter.shooterHeight);

    public static record ShotEntry(Distance targetDistance, AngularVelocity flywheelSpeed,
        Angle exitAngle) {
        public ShotEntry(double distanceFeet, double flywheelSpeed, double hoodAngleDeg) {
            this(Feet.of(distanceFeet), RotationsPerSecond.of(flywheelSpeed),
                Degrees.of(90 - 12.695 - hoodAngleDeg));
        }

        public LinearVelocity theoreticalExitVelocity() {
            return MetersPerSecond.of(Math.sqrt(Math.pow(targetDistance.in(Meters), 2.0) * 9.81
                / (targetDistance.in(Meters) * Math.sin(exitAngle.in(Radians) * 2.0)
                    - 2.0 * ShotData.shooterToTargetHeightDiff.in(Meters)
                        * Math.pow(Math.cos(exitAngle.in(Radians)), 2.0))));
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

    /** Parameters for a single instance of shooting. */
    public static record ShotParameters(double desiredSpeed, double hoodAngleDeg,
        double timeOfFlight, boolean isOkayToShoot) {
    }

    /** Get parameters for a given shot situation. */
    public static ShotParameters getShotParameters(double distance, double currentFlywheelSpeed,
        boolean log) {
        return null;
    }

}
