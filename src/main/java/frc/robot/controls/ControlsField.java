package frc.robot.controls;

import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

/**
 * The set of driver-tunable control parameters.
 *
 * <p>
 * Each constant describes one tunable value: the key it is stored under in {@code profiles.json}
 * and published under on NetworkTables, its factory default, and the range the web tuner clamps it
 * to. A {@link ControlsConfig} is simply a {@code double[]} indexed by {@link #ordinal()}.
 *
 * <p>
 * Adding a value here is the only change required to expose it end to end &mdash; the JSON schema,
 * the NetworkTables layout, the logged inputs, and the web interface are all driven off this enum.
 * Existing profile files remain loadable, since missing keys fall back to {@link #defaultValue()}.
 */
@NullMarked
public enum ControlsField {

    /** Radial deadband applied to the translation stick, as a fraction of full deflection. */
    TRANSLATION_DEADBAND("translationDeadband", 0.10, 0.0, 0.40),
    /** Response curve for translation. 1.0 is linear (MoSim-like), 2.0 is squared. */
    TRANSLATION_EXPONENT("translationExponent", 1.0, 1.0, 3.0),
    /** Stick deflection at which translation saturates, as a fraction of full deflection. */
    TRANSLATION_SATURATION("translationSaturation", 0.95, 0.50, 1.0),
    /** Maximum commanded translational speed, in meters per second. */
    TRANSLATION_MAX_SPEED("translationMaxSpeed", 4.0, 0.5, 7.0),

    /** Deadband applied to the rotation stick, as a fraction of full deflection. */
    ROTATION_DEADBAND("rotationDeadband", 0.10, 0.0, 0.40),
    /** Response curve for rotation. 1.0 is linear, 2.0 is squared. */
    ROTATION_EXPONENT("rotationExponent", 1.0, 1.0, 3.0),
    /** Maximum commanded angular velocity, in radians per second. */
    ROTATION_MAX_SPEED("rotationMaxSpeed", 7.0, 0.5, 10.0),

    /**
     * Time constant of the coast applied when the translation stick returns to centre, in seconds.
     * Zero reproduces the previous behaviour of commanding a hard stop.
     */
    RELEASE_TIME_CONSTANT("releaseTimeConstant", 0.33, 0.0, 1.0),

    /** Maximum commanded translational speed while the shoot trigger is held, in m/s. */
    SHOOT_TRANSLATION_MAX_SPEED("shootTranslationMaxSpeed", 1.0, 0.25, 4.0),
    /** Maximum commanded angular velocity while the shoot trigger is held, in rad/s. */
    SHOOT_ROTATION_MAX_SPEED("shootRotationMaxSpeed", 1.5, 0.25, 6.0),

    /** Acceleration limit along the current direction of travel, in m/s^2. */
    FORWARD_ACCEL_LIMIT("forwardAccelLimit", 10.0, 1.0, 30.0),
    /** Lateral acceleration limit before the command is treated as a skid, in m/s^2. */
    SKID_LIMIT("skidLimit", 1000.0, 1.0, 1000.0),
    /** Forward acceleration limit imposed to avoid tipping, in m/s^2. */
    FORWARD_TILT_LIMIT("forwardTiltLimit", 1000.0, 1.0, 1000.0),
    /** Rearward acceleration limit imposed to avoid tipping, in m/s^2. */
    BACK_TILT_LIMIT("backTiltLimit", 1000.0, 1.0, 1000.0),
    /** Leftward acceleration limit imposed to avoid tipping, in m/s^2. */
    LEFT_TILT_LIMIT("leftTiltLimit", 1000.0, 1.0, 1000.0),
    /** Rightward acceleration limit imposed to avoid tipping, in m/s^2. */
    RIGHT_TILT_LIMIT("rightTiltLimit", 1000.0, 1.0, 1000.0);

    /** Limits at or above this value are treated as disabled by {@code SwerveRateLimiter}. */
    public static final double LIMIT_DISABLED = 800.0;

    private final String key;
    private final double defaultValue;
    private final double minimum;
    private final double maximum;

    ControlsField(String key, double defaultValue, double minimum, double maximum) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.minimum = minimum;
        this.maximum = maximum;
    }

    /** The JSON key and NetworkTables topic name for this value. */
    public String key() {
        return key;
    }

    /** The value used when no profile supplies one. */
    public double defaultValue() {
        return defaultValue;
    }

    /** The smallest value the tuner will accept. */
    public double minimum() {
        return minimum;
    }

    /** The largest value the tuner will accept. */
    public double maximum() {
        return maximum;
    }

    /** Constrain a value to this field's accepted range. */
    public double clamp(double value) {
        if (!Double.isFinite(value)) {
            return defaultValue;
        }
        return Math.max(minimum, Math.min(maximum, value));
    }

    /** A fresh array of every field's default value, indexed by {@link #ordinal()}. */
    public static double[] defaults() {
        ControlsField[] fields = values();
        double[] out = new double[fields.length];
        for (int i = 0; i < fields.length; i++) {
            out[i] = fields[i].defaultValue;
        }
        return out;
    }

    /** Look up a field by its JSON key, or null when the key is not recognised. */
    public static @Nullable ControlsField fromKey(String key) {
        for (ControlsField field : values()) {
            if (field.key.equals(key)) {
                return field;
            }
        }
        return null;
    }

}
