package frc.robot.tuning;

import java.util.Arrays;
import org.jspecify.annotations.NullMarked;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * An immutable snapshot of the drivetrain's tunable gains and geometry.
 *
 * <p>
 * Holds the scalar values of {@link TuningField} plus the four module azimuth offsets, which are
 * captured against the physical robot rather than typed in. Parsing is deliberately forgiving:
 * unknown keys are ignored and missing ones fall back to defaults, so a file written before a
 * field existed still loads and a hand-edited file cannot brick the drivetrain.
 */
@NullMarked
public final class DrivetrainTuning {

    /** Number of swerve modules whose azimuth offsets are stored. */
    public static final int MODULE_COUNT = 4;

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private final double[] values;
    private final double[] moduleOffsets;

    private DrivetrainTuning(double[] values, double[] moduleOffsets) {
        this.values = values;
        this.moduleOffsets = moduleOffsets;
    }

    /** The shipped configuration. */
    public static DrivetrainTuning defaults() {
        return new DrivetrainTuning(TuningField.defaults(), new double[MODULE_COUNT]);
    }

    /**
     * Wrap raw arrays, clamping each value to its field's range.
     *
     * @param raw values indexed by {@link TuningField#ordinal()}
     * @param offsets module azimuth offsets in rotations
     * @return a configuration holding a defensive, validated copy
     */
    public static DrivetrainTuning of(double[] raw, double[] offsets) {
        TuningField[] fields = TuningField.values();
        double[] out = TuningField.defaults();
        for (int i = 0; i < fields.length && i < raw.length; i++) {
            out[i] = fields[i].clamp(raw[i]);
        }
        double[] outOffsets = new double[MODULE_COUNT];
        for (int i = 0; i < MODULE_COUNT && i < offsets.length; i++) {
            outOffsets[i] = Double.isFinite(offsets[i]) ? offsets[i] : 0.0;
        }
        return new DrivetrainTuning(out, outOffsets);
    }

    /** Read a single field. */
    public double get(TuningField field) {
        return values[field.ordinal()];
    }

    /**
     * Return a copy with one field replaced.
     *
     * @param field the field to change
     * @param value the new value, clamped to the field's range
     * @return a new configuration; this one is unchanged
     */
    public DrivetrainTuning with(TuningField field, double value) {
        double[] out = values.clone();
        out[field.ordinal()] = field.clamp(value);
        return new DrivetrainTuning(out, moduleOffsets.clone());
    }

    /** Return a copy with new module azimuth offsets, in rotations. */
    public DrivetrainTuning withModuleOffsets(double[] offsets) {
        double[] out = new double[MODULE_COUNT];
        for (int i = 0; i < MODULE_COUNT && i < offsets.length; i++) {
            out[i] = Double.isFinite(offsets[i]) ? offsets[i] : 0.0;
        }
        return new DrivetrainTuning(values.clone(), out);
    }

    /** A defensive copy of the scalar values, for logging and serialisation. */
    public double[] toArray() {
        return values.clone();
    }

    /** A defensive copy of the module azimuth offsets, in rotations. */
    public double[] moduleOffsets() {
        return moduleOffsets.clone();
    }

    /** Drive static friction feedforward, in volts. */
    public double driveKs() {
        return values[TuningField.DRIVE_KS.ordinal()];
    }

    /** Drive velocity feedforward, in volts per rad/s. */
    public double driveKv() {
        return values[TuningField.DRIVE_KV.ordinal()];
    }

    /** Drive acceleration feedforward, in volts per rad/s^2. */
    public double driveKa() {
        return values[TuningField.DRIVE_KA.ordinal()];
    }

    /** Drive velocity-loop proportional gain. */
    public double driveKp() {
        return values[TuningField.DRIVE_KP.ordinal()];
    }

    /** Drive velocity-loop derivative gain. */
    public double driveKd() {
        return values[TuningField.DRIVE_KD.ordinal()];
    }

    /** Azimuth position-loop proportional gain. */
    public double angleKp() {
        return values[TuningField.ANGLE_KP.ordinal()];
    }

    /** Azimuth position-loop derivative gain. */
    public double angleKd() {
        return values[TuningField.ANGLE_KD.ordinal()];
    }

    /** Effective wheel radius, in inches. */
    public double wheelRadiusInches() {
        return values[TuningField.WHEEL_RADIUS.ordinal()];
    }

    /** Amplitude of the step-response test. */
    public double stepAmplitude() {
        return values[TuningField.STEP_AMPLITUDE.ordinal()];
    }

    /** Hold time for each half of the step-response square wave, in seconds. */
    public double stepPeriod() {
        return values[TuningField.STEP_PERIOD.ordinal()];
    }

    /**
     * Parse a configuration from JSON.
     *
     * @param json the document to read
     * @return the parsed configuration, or defaults if the document is unusable
     */
    public static DrivetrainTuning fromJson(String json) {
        try {
            JsonNode root = MAPPER.readTree(json);
            DrivetrainTuning out = defaults();
            for (TuningField field : TuningField.values()) {
                JsonNode value = root.get(field.key());
                if (value != null && value.isNumber()) {
                    out = out.with(field, value.asDouble());
                }
            }
            JsonNode offsets = root.get("moduleOffsets");
            if (offsets != null && offsets.isArray()) {
                double[] parsed = new double[MODULE_COUNT];
                for (int i = 0; i < MODULE_COUNT && i < offsets.size(); i++) {
                    parsed[i] = offsets.get(i).asDouble();
                }
                out = out.withModuleOffsets(parsed);
            }
            return out;
        } catch (Exception e) {
            System.err.println("[Tuning] could not parse tuning, using defaults: " + e);
            return defaults();
        }
    }

    /** Serialise as pretty-printed JSON suitable for checking into git. */
    public String toJson() {
        ObjectNode root = MAPPER.createObjectNode();
        for (TuningField field : TuningField.values()) {
            root.put(field.key(), get(field));
        }
        ArrayNode offsets = root.putArray("moduleOffsets");
        for (double offset : moduleOffsets) {
            offsets.add(offset);
        }
        try {
            return MAPPER.writerWithDefaultPrettyPrinter().writeValueAsString(root) + "\n";
        } catch (Exception e) {
            System.err.println("[Tuning] could not serialise tuning: " + e);
            return "{}\n";
        }
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof DrivetrainTuning t && Arrays.equals(values, t.values)
            && Arrays.equals(moduleOffsets, t.moduleOffsets);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(values) * 31 + Arrays.hashCode(moduleOffsets);
    }

}
