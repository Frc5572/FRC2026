package frc.robot.controls;

import java.util.Arrays;
import org.jspecify.annotations.NullMarked;

/**
 * An immutable snapshot of every driver-tunable control parameter.
 *
 * <p>
 * Instances are handed to the code that shapes driver input &mdash;
 * {@code TeleopControls} and {@code SwerveRateLimiter} &mdash; rather than those call sites
 * reaching for static constants. This keeps the values explicit at every use, makes them testable
 * without a robot, and guarantees that a replay sees exactly the configuration the log recorded.
 *
 * <p>
 * Values are stored as a {@code double[]} indexed by {@link ControlsField#ordinal()}; the named
 * accessors below are the intended way to read them.
 */
@NullMarked
public final class ControlsConfig {

    private final double[] values;

    private ControlsConfig(double[] values) {
        this.values = values;
    }

    /** The factory-default configuration. */
    public static ControlsConfig defaults() {
        return new ControlsConfig(ControlsField.defaults());
    }

    /**
     * Wrap a raw value array, clamping every entry to its field's accepted range.
     *
     * <p>
     * A short array is padded with defaults, so a profile written before a new field was added
     * still loads.
     *
     * @param raw values indexed by {@link ControlsField#ordinal()}
     * @return a configuration holding a defensive, validated copy of {@code raw}
     */
    public static ControlsConfig of(double[] raw) {
        ControlsField[] fields = ControlsField.values();
        double[] out = ControlsField.defaults();
        for (int i = 0; i < fields.length && i < raw.length; i++) {
            out[i] = fields[i].clamp(raw[i]);
        }
        return new ControlsConfig(out);
    }

    /** Read a single field. */
    public double get(ControlsField field) {
        return values[field.ordinal()];
    }

    /**
     * Return a copy of this configuration with one field replaced.
     *
     * @param field the field to change
     * @param value the new value, clamped to the field's range
     * @return a new configuration; this one is unchanged
     */
    public ControlsConfig with(ControlsField field, double value) {
        double[] out = values.clone();
        out[field.ordinal()] = field.clamp(value);
        return new ControlsConfig(out);
    }

    /** A defensive copy of the backing array, for logging and serialisation. */
    public double[] toArray() {
        return values.clone();
    }

    /** Radial deadband on the translation stick. */
    public double translationDeadband() {
        return values[ControlsField.TRANSLATION_DEADBAND.ordinal()];
    }

    /** Translation response curve exponent; 1.0 is linear. */
    public double translationExponent() {
        return values[ControlsField.TRANSLATION_EXPONENT.ordinal()];
    }

    /** Stick deflection at which translation reaches full output. */
    public double translationSaturation() {
        return values[ControlsField.TRANSLATION_SATURATION.ordinal()];
    }

    /** Maximum commanded translational speed, in meters per second. */
    public double translationMaxSpeed() {
        return values[ControlsField.TRANSLATION_MAX_SPEED.ordinal()];
    }

    /** Deadband on the rotation stick. */
    public double rotationDeadband() {
        return values[ControlsField.ROTATION_DEADBAND.ordinal()];
    }

    /** Rotation response curve exponent; 1.0 is linear. */
    public double rotationExponent() {
        return values[ControlsField.ROTATION_EXPONENT.ordinal()];
    }

    /** Maximum commanded angular velocity, in radians per second. */
    public double rotationMaxSpeed() {
        return values[ControlsField.ROTATION_MAX_SPEED.ordinal()];
    }

    /** Coast time constant applied when the translation stick is released, in seconds. */
    public double releaseTimeConstant() {
        return values[ControlsField.RELEASE_TIME_CONSTANT.ordinal()];
    }

    /** Maximum commanded translational speed while shooting, in meters per second. */
    public double shootTranslationMaxSpeed() {
        return values[ControlsField.SHOOT_TRANSLATION_MAX_SPEED.ordinal()];
    }

    /** Maximum commanded angular velocity while shooting, in radians per second. */
    public double shootRotationMaxSpeed() {
        return values[ControlsField.SHOOT_ROTATION_MAX_SPEED.ordinal()];
    }

    /** Acceleration limit along the current direction of travel, in m/s^2. */
    public double forwardAccelLimit() {
        return values[ControlsField.FORWARD_ACCEL_LIMIT.ordinal()];
    }

    /** Lateral acceleration limit before the command is treated as a skid, in m/s^2. */
    public double skidLimit() {
        return values[ControlsField.SKID_LIMIT.ordinal()];
    }

    /** Forward tipping acceleration limit, in m/s^2. */
    public double forwardTiltLimit() {
        return values[ControlsField.FORWARD_TILT_LIMIT.ordinal()];
    }

    /** Rearward tipping acceleration limit, in m/s^2. */
    public double backTiltLimit() {
        return values[ControlsField.BACK_TILT_LIMIT.ordinal()];
    }

    /** Leftward tipping acceleration limit, in m/s^2. */
    public double leftTiltLimit() {
        return values[ControlsField.LEFT_TILT_LIMIT.ordinal()];
    }

    /** Rightward tipping acceleration limit, in m/s^2. */
    public double rightTiltLimit() {
        return values[ControlsField.RIGHT_TILT_LIMIT.ordinal()];
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ControlsConfig cfg && Arrays.equals(values, cfg.values);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(values);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("ControlsConfig{");
        ControlsField[] fields = ControlsField.values();
        for (int i = 0; i < fields.length; i++) {
            if (i > 0) {
                sb.append(", ");
            }
            sb.append(fields[i].key()).append('=').append(values[i]);
        }
        return sb.append('}').toString();
    }

}
