package frc.robot.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

/** Tests for control configuration validation and profile serialisation. */
public class ControlsConfigTest {

    private static final double EPS = 1e-9;

    /** Out-of-range values are pulled back to the field's limits rather than accepted. */
    @Test
    public void clampsOutOfRangeValues() {
        ControlsConfig cfg = ControlsConfig.defaults()
            .with(ControlsField.TRANSLATION_MAX_SPEED, 999.0)
            .with(ControlsField.TRANSLATION_DEADBAND, -5.0);
        assertEquals(ControlsField.TRANSLATION_MAX_SPEED.maximum(), cfg.translationMaxSpeed(), EPS);
        assertEquals(ControlsField.TRANSLATION_DEADBAND.minimum(), cfg.translationDeadband(), EPS);
    }

    /** A non-finite value must not reach the drivetrain. */
    @Test
    public void rejectsNonFiniteValues() {
        ControlsConfig cfg = ControlsConfig.defaults()
            .with(ControlsField.ROTATION_MAX_SPEED, Double.NaN);
        assertEquals(ControlsField.ROTATION_MAX_SPEED.defaultValue(), cfg.rotationMaxSpeed(), EPS);
    }

    /** A profile written before a field existed still loads, with the new field defaulted. */
    @Test
    public void padsShortArraysWithDefaults() {
        ControlsConfig cfg = ControlsConfig.of(new double[] {0.2});
        assertEquals(0.2, cfg.translationDeadband(), EPS);
        assertEquals(ControlsField.ROTATION_MAX_SPEED.defaultValue(), cfg.rotationMaxSpeed(), EPS);
    }

    /** {@code with} must not mutate the receiver. */
    @Test
    public void withIsImmutable() {
        ControlsConfig original = ControlsConfig.defaults();
        double before = original.translationMaxSpeed();
        ControlsConfig changed = original.with(ControlsField.TRANSLATION_MAX_SPEED, 2.0);
        assertEquals(before, original.translationMaxSpeed(), EPS);
        assertEquals(2.0, changed.translationMaxSpeed(), EPS);
    }

    /** Every field must have a distinct key and a default inside its own range. */
    @Test
    public void fieldMetadataIsConsistent() {
        for (ControlsField field : ControlsField.values()) {
            assertTrue(field.minimum() <= field.defaultValue(),
                field.key() + " default below minimum");
            assertTrue(field.defaultValue() <= field.maximum(),
                field.key() + " default above maximum");
            assertEquals(field, ControlsField.fromKey(field.key()));
        }
    }

    /** Profiles survive a JSON round trip unchanged. */
    @Test
    public void profilesRoundTripThroughJson() {
        ControlsProfiles profiles = new ControlsProfiles();
        profiles.put("driver", ControlsConfig.defaults()
            .with(ControlsField.TRANSLATION_MAX_SPEED, 3.25)
            .with(ControlsField.RELEASE_TIME_CONSTANT, 0.5));

        ControlsProfiles parsed = ControlsProfiles.fromJson(profiles.toJson());
        assertEquals("driver", parsed.activeName());
        assertEquals(3.25, parsed.active().translationMaxSpeed(), EPS);
        assertEquals(0.5, parsed.active().releaseTimeConstant(), EPS);
        assertTrue(parsed.names().contains(ControlsProfiles.DEFAULT_PROFILE));
    }

    /** Unknown keys are ignored and missing ones fall back to defaults. */
    @Test
    public void toleratesPartialAndUnknownKeys() {
        String json = "{\"active\":\"a\",\"profiles\":{\"a\":"
            + "{\"translationMaxSpeed\":2.5,\"somethingElse\":7}}}";
        ControlsProfiles parsed = ControlsProfiles.fromJson(json);
        assertEquals(2.5, parsed.active().translationMaxSpeed(), EPS);
        assertEquals(ControlsField.ROTATION_MAX_SPEED.defaultValue(),
            parsed.active().rotationMaxSpeed(), EPS);
    }

    /** A corrupt file must not stop the robot from driving. */
    @Test
    public void malformedJsonFallsBackToDefaults() {
        ControlsProfiles parsed = ControlsProfiles.fromJson("{ this is not json");
        assertEquals(ControlsProfiles.DEFAULT_PROFILE, parsed.activeName());
        assertEquals(ControlsField.TRANSLATION_MAX_SPEED.defaultValue(),
            parsed.active().translationMaxSpeed(), EPS);
    }

    /** An active name that no longer exists falls back to a real profile. */
    @Test
    public void unknownActiveProfileFallsBack() {
        String json = "{\"active\":\"ghost\",\"profiles\":{\"a\":{\"translationMaxSpeed\":2.0}}}";
        ControlsProfiles parsed = ControlsProfiles.fromJson(json);
        assertEquals("a", parsed.activeName());
    }

    /** The last remaining profile cannot be deleted. */
    @Test
    public void cannotDeleteLastProfile() {
        ControlsProfiles profiles = new ControlsProfiles();
        assertFalse(profiles.remove(ControlsProfiles.DEFAULT_PROFILE));
        profiles.put("second", ControlsConfig.defaults());
        assertTrue(profiles.remove("second"));
    }

}
