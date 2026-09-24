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
    private final boolean[] enabled;
    private final ControlsCurve translationCurve;
    private final ControlsCurve rotationCurve;
    private final ControlScheme scheme;

    private ControlsConfig(double[] values, boolean[] enabled, ControlsCurve translationCurve,
        ControlsCurve rotationCurve, ControlScheme scheme) {
        this.values = values;
        this.enabled = enabled;
        this.translationCurve = translationCurve;
        this.rotationCurve = rotationCurve;
        this.scheme = scheme;
    }

    /** The factory-default configuration. */
    public static ControlsConfig defaults() {
        return new ControlsConfig(ControlsField.defaults(),
            ControlsField.defaultEnabledFlags(), ControlsCurve.power(), ControlsCurve.power(),
            ControlScheme.defaultScheme());
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
        return of(raw, ControlsField.defaultEnabledFlags(), ControlsCurve.power(),
            ControlsCurve.power());
    }

    /**
     * Wrap a raw value array and a pair of response curves.
     *
     * @param raw values indexed by {@link ControlsField#ordinal()}
     * @param translationCurve the translation response curve
     * @param rotationCurve the rotation response curve
     * @return a configuration holding a defensive, validated copy of the inputs
     */
    public static ControlsConfig of(double[] raw, boolean[] rawEnabled,
        ControlsCurve translationCurve, ControlsCurve rotationCurve) {
        ControlsField[] fields = ControlsField.values();
        double[] out = ControlsField.defaults();
        boolean[] flags = ControlsField.defaultEnabledFlags();
        for (int i = 0; i < fields.length && i < raw.length; i++) {
            out[i] = fields[i].clamp(raw[i]);
        }
        for (int i = 0; i < fields.length && i < rawEnabled.length; i++) {
            flags[i] = !fields[i].canDisable() || rawEnabled[i];
        }
        return new ControlsConfig(out, flags, translationCurve, rotationCurve,
            ControlScheme.defaultScheme());
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
        return new ControlsConfig(out, enabled.clone(), translationCurve, rotationCurve, scheme);
    }

    /**
     * Return a copy of this configuration with one field switched on or off.
     *
     * <p>
     * A field that cannot be disabled ignores the request and stays on.
     *
     * @param field the field to toggle
     * @param on whether the field should take effect
     * @return a new configuration; this one is unchanged
     */
    public ControlsConfig withEnabled(ControlsField field, boolean on) {
        boolean[] out = enabled.clone();
        out[field.ordinal()] = !field.canDisable() || on;
        return new ControlsConfig(values.clone(), out, translationCurve, rotationCurve, scheme);
    }

    /** Whether the given field currently takes effect. */
    public boolean isEnabled(ControlsField field) {
        return enabled[field.ordinal()];
    }

    /** A defensive copy of the enabled flags, for logging and serialisation. */
    public boolean[] enabledFlags() {
        return enabled.clone();
    }

    /**
     * The effective value of a limit field: its configured value, or
     * {@link ControlsField#LIMIT_DISABLED_VALUE} when it has been switched off.
     *
     * @param field the limit to read
     * @return the value the drivetrain should apply
     */
    public double limit(ControlsField field) {
        return enabled[field.ordinal()] ? values[field.ordinal()]
            : ControlsField.LIMIT_DISABLED_VALUE;
    }

    /** A copy of this configuration with a different translation response curve. */
    public ControlsConfig withTranslationCurve(ControlsCurve curve) {
        return new ControlsConfig(values.clone(), enabled.clone(), curve, rotationCurve, scheme);
    }

    /** A copy of this configuration with a different rotation response curve. */
    public ControlsConfig withRotationCurve(ControlsCurve curve) {
        return new ControlsConfig(values.clone(), enabled.clone(), translationCurve, curve, scheme);
    }

    /** A copy of this configuration using a different set of driver bindings. */
    public ControlsConfig withScheme(ControlScheme newScheme) {
        return new ControlsConfig(values.clone(), enabled.clone(), translationCurve,
            rotationCurve, newScheme);
    }

    /** Which set of driver bindings is in force. */
    public ControlScheme scheme() {
        return scheme;
    }

    /** The translation response curve. */
    public ControlsCurve translationCurve() {
        return translationCurve;
    }

    /** The rotation response curve. */
    public ControlsCurve rotationCurve() {
        return rotationCurve;
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
        return limit(ControlsField.FORWARD_ACCEL_LIMIT);
    }

    /** Lateral acceleration limit before the command is treated as a skid, in m/s^2. */
    public double skidLimit() {
        return limit(ControlsField.SKID_LIMIT);
    }

    /** Forward tipping acceleration limit, in m/s^2. */
    public double forwardTiltLimit() {
        return limit(ControlsField.FORWARD_TILT_LIMIT);
    }

    /** Rearward tipping acceleration limit, in m/s^2. */
    public double backTiltLimit() {
        return limit(ControlsField.BACK_TILT_LIMIT);
    }

    /** Leftward tipping acceleration limit, in m/s^2. */
    public double leftTiltLimit() {
        return limit(ControlsField.LEFT_TILT_LIMIT);
    }

    /** Rightward tipping acceleration limit, in m/s^2. */
    public double rightTiltLimit() {
        return limit(ControlsField.RIGHT_TILT_LIMIT);
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ControlsConfig cfg && Arrays.equals(values, cfg.values)
            && Arrays.equals(enabled, cfg.enabled)
            && translationCurve.equals(cfg.translationCurve)
            && rotationCurve.equals(cfg.rotationCurve) && scheme == cfg.scheme;
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(values) * 31 + translationCurve.hashCode();
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
