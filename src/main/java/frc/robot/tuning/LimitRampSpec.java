package frc.robot.tuning;

import java.util.function.DoubleSupplier;
import org.jspecify.annotations.NullMarked;
import frc.robot.controls.ControlsField;

/**
 * How one acceleration-limit procedure ramps and what counts as failure.
 *
 * <p>
 * The three procedures in software-sessions.md (lines 157&ndash;176) differ only in which limit
 * they move, which way they move it, and which detector they watch, so one parameterised ramp
 * covers all three. The factory methods below encode the doc's own instructions.
 *
 * @param field the limit being tuned
 * @param detector reads the failure condition; see {@link LimitDetectors}
 * @param start first candidate value
 * @param step change applied between bursts; negative ramps downward
 * @param threshold detector value that counts as failure
 * @param trippedWhenAbove true if exceeding the threshold is failure, false if staying at or
 *        below it is the success the ramp is searching for
 * @param burstSeconds how long each acceleration burst lasts
 * @param settleSeconds pause between bursts, for the robot and the operator
 * @param othersToOpen limits opened to their maximum for the duration, so the limit under test is
 *        the binding one
 */
@NullMarked
public record LimitRampSpec(ControlsField field, DoubleSupplier detector, double start,
    double step, double threshold, boolean trippedWhenAbove, double burstSeconds,
    double settleSeconds, ControlsField[] othersToOpen) {

    /**
     * "Set to a logical value (around 10m/s^2), increase maxAcc until robot can't follow wanted
     * velocity" (doc lines 162&ndash;164).
     *
     * @param followError reads the commanded-versus-measured shortfall, in m/s
     * @return the forward-limit ramp
     */
    public static LimitRampSpec forward(DoubleSupplier followError) {
        return new LimitRampSpec(ControlsField.FORWARD_ACCEL_LIMIT, followError, 10.0, 2.5, 0.5,
            true, 1.2, 1.5, new ControlsField[] {ControlsField.SKID_LIMIT,
                ControlsField.FORWARD_TILT_LIMIT, ControlsField.BACK_TILT_LIMIT});
    }

    /**
     * "Set to logical value (depending on your CG), increase until robot starts tipping" (doc
     * lines 169&ndash;171).
     *
     * <p>
     * The threshold is deliberately a few degrees, not an actual tip. A robot about to go over
     * lifts on one side first, so this stops the ramp while the wheels are only just unloading.
     *
     * @param tiltDegrees reads the chassis tilt from level
     * @return the tilt-limit ramp
     */
    public static LimitRampSpec tilt(DoubleSupplier tiltDegrees) {
        return new LimitRampSpec(ControlsField.FORWARD_TILT_LIMIT, tiltDegrees, 8.0, 2.0, 4.0,
            true, 1.2, 2.0, new ControlsField[] {ControlsField.FORWARD_ACCEL_LIMIT,
                ControlsField.SKID_LIMIT});
    }

    /**
     * "Decrease until robot stops skidding" (doc line 176).
     *
     * <p>
     * Ramps downward, so the answer is the first value that comes back clean rather than the last
     * one before failure.
     *
     * @param skidRatio reads the module max-to-median translational ratio
     * @return the skid-limit ramp
     */
    public static LimitRampSpec skid(DoubleSupplier skidRatio) {
        return new LimitRampSpec(ControlsField.SKID_LIMIT, skidRatio, 40.0, -2.5, 1.25, false,
            1.2, 1.5, new ControlsField[] {ControlsField.FORWARD_ACCEL_LIMIT,
                ControlsField.FORWARD_TILT_LIMIT, ControlsField.BACK_TILT_LIMIT});
    }

}
