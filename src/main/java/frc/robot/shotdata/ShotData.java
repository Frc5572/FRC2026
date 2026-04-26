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
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.math.interp2d.MulAdd;

/**
 * Stores and interpolates shooter parameters for FRC robot shooting mechanics.
 *
 * <p>
 * Contains manually acquired shot data entries and provides interpolation functions for computing
 * optimal flywheel speed, hood angle, and time of flight for a given target distance and current
 * flywheel speed.
 *
 * <p>
 * Three interpolation modes are supported:
 * <ul>
 * <li>{@link #shootFunc} - shooting into the hub</li>
 * <li>{@link #passFunc} - passing along the ground</li>
 * <li>{@link #simFunc} - simulation mode (uses ground entries)</li>
 * </ul>
 */
public class ShotData {

    /**
     * Manually acquired shot data entries used to seed the interpolation tables.
     *
     * <p>
     * Each entry specifies a target distance, flywheel speed, hood angle, and time of flight.
     * Time-of-flight values do not need to be precise.
     */
    public static ShotEntry[] entries = new ShotEntry[] {
        // @formatter:off
        // new ShotEntry(7.21, 45, 18),
        // new ShotEntry(8.0, 46, 20),
        // new ShotEntry(9.05, 50, 20),
        // new ShotEntry(10.13, 51, 20),
        // new ShotEntry(11.10, 54, 20),
        // new ShotEntry(12.05, 56, 20),
        // new ShotEntry(12.82, 58, 20),
        // new ShotEntry(13.87, 61, 21),
        // new ShotEntry(15.01, 64, 22),
        // new ShotEntry(16.14, 67, 23),
        // new ShotEntry(7.04, 43, 15),
        // new ShotEntry(8.00, 44, 15),
        // new ShotEntry(9.18, 45, 15),
        // new ShotEntry(10.6, 48, 20),
        // new ShotEntry(12.46, 53, 20),
        // new ShotEntry(13.67, 54, 22),
        // new ShotEntry(14.45, 56, 24),
        // new ShotEntry(15.75, 57, 24.5),
        // new ShotEntry(16.49, 57.5, 28),
        // new ShotEntry(18.15, 61, 28),
        // new ShotEntry(19.10, 63, 28),
        // new ShotEntry(20.2, 65, 28),
        // new ShotEntry(21.3, 67, 28),
        // new ShotEntry(22.5, 69, 28),

        new ShotEntry(4.04, 37, 2),
        new ShotEntry(5.8, 37, 7),
        new ShotEntry(7.15, 39, 12),
        new ShotEntry(8.42, 40, 15),
        new ShotEntry(9.58, 43, 16),
        new ShotEntry(10.87, 45, 17),
        new ShotEntry(11.41, 46, 18),
        new ShotEntry(12.6, 48, 18.5),
        new ShotEntry(13.4, 50, 19),
        new ShotEntry(14.68, 51.5, 19.5),
        new ShotEntry(14.68, 51.5, 19.5),
        new ShotEntry(15.7, 53, 20),
        new ShotEntry(16.49, 55, 21),
        new ShotEntry(18.0, 57, 22),

        // @formatter:on
    };

    /**
     * The vertical distance from the shooter to the hub target center point.
     *
     * <p>
     * Computed as the difference between the hub's top center Z coordinate and the configured
     * shooter height from {@link frc.robot.Constants}.
     */
    public static final Distance shooterToTargetHeightDiff =
        FieldConstants.Hub.topCenterPoint.getMeasureZ().minus(Constants.Shooter.shooterHeight);

    /**
     * Represents a single data point mapping a target distance to shooter parameters.
     *
     * <p>
     * Can be constructed either with typed unit measures or with raw primitive values (feet,
     * rotations per second, degrees, seconds) for convenience.
     */
    public static record ShotEntry(Distance targetDistance, AngularVelocity flywheelSpeed,
        Angle exitAngle, LinearVelocity exitVelocity, Time tof) {
        /**
         * Convenience constructor using raw primitive values.
         *
         * @param distanceFeet target distance in feet
         * @param flywheelSpeed flywheel speed in rotations per second
         * @param hoodAngleDeg hood angle in degrees (converted internally to exit angle)
         * @param tof time of flight in seconds
         */
        public ShotEntry(double distanceFeet, double flywheelSpeed, double hoodAngleDeg) {
            this(Feet.of(distanceFeet), RotationsPerSecond.of(flywheelSpeed),
                Degrees.of(90 - 12.695 - hoodAngleDeg), MetersPerSecond.of(0.0), Seconds.of(0.0));
        }

        /**
         * Estimates the backspin applied to the ball based on the current hood angle and flywheel
         * speed, using a generated lookup table.
         *
         * @return estimated backspin value
         */
        public double backspin() {
            return GeneratedLUTs.estimatedBackspin(hoodAngle().in(Degrees),
                flywheelSpeed().in(RotationsPerSecond));
        }

        /**
         * Computes the theoretically required ball exit velocity using projectile motion,
         * accounting for the height difference between the shooter and the hub target.
         *
         * @return theoretical exit velocity in meters per second
         */
        public LinearVelocity theoreticalExitVelocity() {
            return MetersPerSecond.of(Math.sqrt(Math.pow(targetDistance.in(Meters), 2.0) * 9.81
                / (targetDistance.in(Meters) * Math.sin(exitAngle.in(Radians) * 2.0)
                    - 2.0 * ShotData.shooterToTargetHeightDiff.in(Meters)
                        * Math.pow(Math.cos(exitAngle.in(Radians)), 2.0))));
        }

        /**
         * Back-calculates the ball exit velocity from the recorded time of flight and exit angle,
         * without relying on physics modeling.
         *
         * @return back-traced exit velocity in meters per second
         */
        public LinearVelocity backtracedExitVelocity() {
            return MetersPerSecond.of(
                targetDistance.in(Meters) / Math.cos(exitAngle.in(Radians)) / tof().in(Seconds));
        }

        /**
         * Computes the theoretical time of flight using the exit angle and
         * {@link #theoreticalExitVelocity()}.
         *
         * @return theoretical time of flight in seconds
         */
        public Time theoreticalTimeOfFlight() {
            return Seconds.of(targetDistance.in(Meters) / Math.cos(exitAngle.in(Radians))
                / theoreticalExitVelocity().in(MetersPerSecond));
        }

        /**
         * Computes the ball exit velocity assuming no slip between the flywheel and the ball (i.e.
         * the ball surface speed equals the wheel surface speed).
         *
         * @return no-slip exit velocity in meters per second
         */
        public LinearVelocity noSlipExitVelocity() {
            return MetersPerSecond.of(flywheelSpeed.in(RadiansPerSecond) * Inches.of(2).in(Meters));
        }

        /**
         * Scales the no-slip exit velocity by a pre-calculated speed transfer coefficient to
         * account for energy losses during ball compression and release.
         *
         * @return speed-transfer-adjusted exit velocity in meters per second
         */
        public LinearVelocity speedTransferExitVelocity() {
            return noSlipExitVelocity().times(GeneratedLUTs.SPEED_TRANSFER_COEFF);
        }

        /**
         * Converts the stored exit angle back to a hood angle in degrees, reversing the offset
         * applied during construction.
         *
         * @return hood angle in degrees
         */
        public Angle hoodAngle() {
            return Degrees.of(90 - 12.695 - exitAngle.in(Degrees));
        }
    }

    /**
     * Defines addition and scalar multiplication over {@link ShotEntry} objects, enabling weighted
     * interpolation between entries.
     */
    public static final MulAdd<ShotEntry> mulAdd = new MulAdd<ShotEntry>() {

        @Override
        public ShotEntry mul(ShotEntry a, double b) {
            return new ShotEntry(a.targetDistance.times(b), a.flywheelSpeed.times(b),
                a.exitAngle.times(b), a.exitVelocity.times(b), a.tof.times(b));
        }

        @Override
        public ShotEntry add(ShotEntry a, ShotEntry b) {
            return new ShotEntry(a.targetDistance.plus(b.targetDistance),
                a.flywheelSpeed.plus(b.flywheelSpeed), a.exitAngle.plus(b.exitAngle),
                a.exitVelocity.plus(b.exitVelocity), a.tof.plus(b.tof));
        }

    };

    /**
     * Interpolation function for simulation. Uses the same ground-pass entries as
     * {@link #passFunc}.
     */
    public static final SemiGriddedBilinearInterpolation<ShotEntry> simFunc =
        new SemiGriddedBilinearInterpolation<>(2.0, GeneratedLUTs.groundEntries, mulAdd,
            x -> x.flywheelSpeed().in(RotationsPerSecond), x -> x.hoodAngle().in(Degrees));

    private static final InterpolatingTreeMap<Double, ShotEntry> shotMap =
        new InterpolatingTreeMap<>((a, b, q) -> (q - a) / (b - a),
            (a, b, t) -> mulAdd.add(mulAdd.mul(a, 1.0 - t), mulAdd.mul(b, t)));
    private static final InterpolatingTreeMap<Double, ShotEntry> passMap =
        new InterpolatingTreeMap<>((a, b, q) -> (q - a) / (b - a),
            (a, b, t) -> mulAdd.add(mulAdd.mul(a, 1.0 - t), mulAdd.mul(b, t)));

    static {
        for (var entry : GeneratedLUTs2.hubEntries) {
            shotMap.put(entry.targetDistance().in(Meters), entry);
        }
        for (var entry : GeneratedLUTs2.groundEntries) {
            passMap.put(entry.targetDistance().in(Meters), entry);
        }
    }

    /**
     * Encapsulates the computed shooter parameters for a single shot instance.
     *
     * @param desiredSpeed target flywheel speed in rotations per second
     * @param hoodAngleDeg hood angle in degrees
     * @param timeOfFlight estimated time of flight in seconds
     * @param isOkayToShoot whether the flywheel is spun up sufficiently to shoot
     */
    public static record ShotParameters(double desiredSpeed, double hoodAngleDeg,
        double timeOfFlight, boolean isOkayToShoot) {
    }

    /**
     * Computes shooter parameters for a hub shot given the current robot state.
     *
     * <p>
     * Interpolates the hood angle and time of flight from the shot table, and looks up the desired
     * flywheel speed.
     *
     * @param distance distance to the hub target in meters
     * @param currentFlywheelSpeed current flywheel speed in rotations per second
     * @param log if {@code true}, logs all parameters via AdvantageKit
     * @return computed {@link ShotParameters} for this shot
     */
    public static ShotParameters getShotParameters(double distance, double currentFlywheelSpeed,
        boolean log) {
        var res = shotMap.get(distance);
        double desiredSpeed = res.flywheelSpeed().in(RotationsPerSecond) + 1;
        double hoodAngleDeg = res.hoodAngle().in(Degrees);
        double tof = res.tof().in(Seconds);
        boolean isOkay = currentFlywheelSpeed > desiredSpeed - 6;
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

    /**
     * Computes shooter parameters for a ground pass given the current robot state.
     *
     * <p>
     * Behaves identically to {@link #getShotParameters} but uses {@link #passFunc} for
     * interpolation, targeting ground-pass trajectories.
     *
     * @param distance distance to the pass target in meters
     * @param currentFlywheelSpeed current flywheel speed in rotations per second
     * @param log if {@code true}, logs all parameters via AdvantageKit
     * @return computed {@link ShotParameters} for this pass
     */
    public static ShotParameters getPassParameters(double distance, double currentFlywheelSpeed,
        boolean log) {
        var res = passMap.get(distance);
        double desiredSpeed = res.flywheelSpeed().in(RotationsPerSecond);
        double hoodAngleDeg = res.hoodAngle().in(Degrees);
        double tof = res.tof().in(Seconds);
        boolean isOkay = currentFlywheelSpeed > desiredSpeed - 10;
        if (distance > FieldConstants.fieldLength / 2.0) {
            desiredSpeed = 180.0;
            hoodAngleDeg = 43.0;
        }
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
