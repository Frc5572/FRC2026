package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

/** Tests for drivetrain tuning validation and serialisation. */
public class DrivetrainTuningTest {

    private static final double EPS = 1e-9;

    /** Out-of-range gains are clamped rather than reaching the motor controllers. */
    @Test
    public void clampsOutOfRangeGains() {
        DrivetrainTuning t = DrivetrainTuning.defaults()
            .with(TuningField.DRIVE_KP, 999.0)
            .with(TuningField.DRIVE_KV, -1.0);
        assertEquals(TuningField.DRIVE_KP.maximum(), t.driveKp(), EPS);
        assertEquals(TuningField.DRIVE_KV.minimum(), t.driveKv(), EPS);
    }

    /** A non-finite gain must never reach a motor controller. */
    @Test
    public void rejectsNonFinite() {
        DrivetrainTuning t = DrivetrainTuning.defaults().with(TuningField.DRIVE_KS, Double.NaN);
        assertEquals(TuningField.DRIVE_KS.defaultValue(), t.driveKs(), EPS);
    }

    /** Every field's default must sit inside its own range and have a unique key. */
    @Test
    public void fieldMetadataIsConsistent() {
        for (TuningField field : TuningField.values()) {
            assertTrue(field.minimum() <= field.defaultValue(),
                field.key() + " default below minimum");
            assertTrue(field.defaultValue() <= field.maximum(),
                field.key() + " default above maximum");
            assertEquals(field, TuningField.fromKey(field.key()));
        }
    }

    /** The shipped kV default is the value the logs flagged, so re-characterizing is visible. */
    @Test
    public void shippedKvIsTheValueUnderSuspicion() {
        assertEquals(0.1515, TuningField.DRIVE_KV.defaultValue(), 1e-4);
        // The no-load value measured from match logs must be reachable by the tuner.
        assertTrue(TuningField.DRIVE_KV.minimum() <= 0.1048
            && 0.1048 <= TuningField.DRIVE_KV.maximum(),
            "the measured kV must be within the tuner's range");
    }

    /** Values and module offsets survive a JSON round trip. */
    @Test
    public void roundTripsThroughJson() {
        DrivetrainTuning t = DrivetrainTuning.defaults()
            .with(TuningField.DRIVE_KV, 0.1048)
            .with(TuningField.DRIVE_KP, 0.05)
            .withModuleOffsets(new double[] {0.1, -0.2, 0.3, 0.4});
        DrivetrainTuning parsed = DrivetrainTuning.fromJson(t.toJson());
        assertEquals(0.1048, parsed.driveKv(), EPS);
        assertEquals(0.05, parsed.driveKp(), EPS);
        assertEquals(t, parsed);
        assertEquals(0.3, parsed.moduleOffsets()[2], EPS);
    }

    /** Unknown keys are ignored and missing ones fall back to defaults. */
    @Test
    public void toleratesPartialAndUnknownKeys() {
        DrivetrainTuning t = DrivetrainTuning.fromJson("{\"driveKv\":0.12,\"nonsense\":5}");
        assertEquals(0.12, t.driveKv(), EPS);
        assertEquals(TuningField.DRIVE_KP.defaultValue(), t.driveKp(), EPS);
    }

    /** A corrupt file must not stop the drivetrain from running. */
    @Test
    public void malformedJsonFallsBackToDefaults() {
        assertEquals(DrivetrainTuning.defaults(), DrivetrainTuning.fromJson("{ not json"));
    }

    /** Mutators do not disturb the receiver. */
    @Test
    public void mutatorsAreImmutable() {
        DrivetrainTuning original = DrivetrainTuning.defaults();
        double before = original.driveKp();
        DrivetrainTuning changed = original.with(TuningField.DRIVE_KP, 0.5);
        assertEquals(before, original.driveKp(), EPS);
        assertEquals(0.5, changed.driveKp(), EPS);
        original.withModuleOffsets(new double[] {1, 1, 1, 1});
        assertEquals(0.0, original.moduleOffsets()[0], EPS);
    }

}
