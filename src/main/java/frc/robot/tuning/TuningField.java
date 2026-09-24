package frc.robot.tuning;

import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

/**
 * The set of live-tunable drivetrain parameters.
 *
 * <p>
 * Each constant names one value, its shipped default, and the range the tuner clamps it to. A
 * {@link DrivetrainTuning} is a {@code double[]} indexed by {@link #ordinal()}, exactly as
 * {@code ControlsField} backs a driver profile &mdash; the two follow the same shape so the
 * NetworkTables layout, the logged inputs and the web page are each one loop over an enum.
 *
 * <p>
 * Defaults here are the values the robot shipped with, not the values it should have. In
 * particular {@link #DRIVE_KV} is the configured 0.1515, which match logs show is about 1.45x the
 * no-load value &mdash; re-running the feedforward characterization is the point of this package.
 */
@NullMarked
public enum TuningField {

    /** Drive static friction feedforward, in volts. */
    DRIVE_KS("driveKs", 0.251, 0.0, 2.0, "V"),
    /** Drive velocity feedforward, in volts per rad/s at the wheel. */
    DRIVE_KV("driveKv", 0.1515, 0.0, 0.5, "V/(rad/s)"),
    /** Drive acceleration feedforward, in volts per rad/s^2 at the wheel. */
    DRIVE_KA("driveKa", 0.0, 0.0, 0.2, "V/(rad/s^2)"),
    /** Drive velocity-loop proportional gain. */
    DRIVE_KP("driveKp", 0.0012, 0.0, 1.0, ""),
    /** Drive velocity-loop derivative gain. */
    DRIVE_KD("driveKd", 0.0, 0.0, 1.0, ""),

    /** Azimuth position-loop proportional gain. */
    ANGLE_KP("angleKp", 100.0, 0.0, 400.0, ""),
    /** Azimuth position-loop derivative gain. */
    ANGLE_KD("angleKd", 0.0, 0.0, 20.0, ""),

    /** Effective wheel radius, in inches. Output of the wheel-radius characterization. */
    WHEEL_RADIUS("wheelRadiusInches", 1.906, 1.5, 2.5, "in"),

    /** Amplitude of the step-response test, in m/s for drive and degrees for azimuth. */
    STEP_AMPLITUDE("stepAmplitude", 2.0, 0.25, 4.0, ""),
    /** How long each half of the step-response square wave is held, in seconds. */
    STEP_PERIOD("stepPeriod", 1.5, 0.5, 5.0, "s");

    private final String key;
    private final double defaultValue;
    private final double minimum;
    private final double maximum;
    private final String units;

    TuningField(String key, double defaultValue, double minimum, double maximum, String units) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.minimum = minimum;
        this.maximum = maximum;
        this.units = units;
    }

    /** The JSON key and NetworkTables topic name for this value. */
    public String key() {
        return key;
    }

    /** The value used when the stored configuration does not supply one. */
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

    /** Units label shown in the tuner, possibly empty. */
    public String units() {
        return units;
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
        TuningField[] fields = values();
        double[] out = new double[fields.length];
        for (int i = 0; i < fields.length; i++) {
            out[i] = fields[i].defaultValue;
        }
        return out;
    }

    /** Look up a field by its JSON key, or null when the key is not recognised. */
    public static @Nullable TuningField fromKey(String key) {
        for (TuningField field : values()) {
            if (field.key.equals(key)) {
                return field;
            }
        }
        return null;
    }

}
