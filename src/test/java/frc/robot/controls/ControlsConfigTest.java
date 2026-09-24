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

    /** A disabled limit reports the sentinel, so the rate limiter skips that stage. */
    @Test
    public void disabledLimitReportsSentinel() {
        ControlsConfig cfg = ControlsConfig.defaults()
            .with(ControlsField.FORWARD_ACCEL_LIMIT, 12.0);
        assertEquals(12.0, cfg.forwardAccelLimit(), EPS);
        assertTrue(cfg.forwardAccelLimit() < ControlsField.LIMIT_DISABLED);

        ControlsConfig off = cfg.withEnabled(ControlsField.FORWARD_ACCEL_LIMIT, false);
        assertEquals(ControlsField.LIMIT_DISABLED_VALUE, off.forwardAccelLimit(), EPS);
        assertTrue(off.forwardAccelLimit() >= ControlsField.LIMIT_DISABLED,
            "a disabled limit must read as disabled to the rate limiter");
        // The underlying value is kept, so re-enabling restores what the driver had set.
        assertEquals(12.0, off.get(ControlsField.FORWARD_ACCEL_LIMIT), EPS);
        assertEquals(12.0,
            off.withEnabled(ControlsField.FORWARD_ACCEL_LIMIT, true).forwardAccelLimit(), EPS);
    }

    /** Fields that cannot be disabled ignore the request. */
    @Test
    public void nonDisableableFieldsStayOn() {
        ControlsConfig cfg =
            ControlsConfig.defaults().withEnabled(ControlsField.TRANSLATION_MAX_SPEED, false);
        assertTrue(cfg.isEnabled(ControlsField.TRANSLATION_MAX_SPEED));
        assertEquals(ControlsField.TRANSLATION_MAX_SPEED.defaultValue(),
            cfg.translationMaxSpeed(), EPS);
    }

    /** Every limit lives in the normalised 5-50 range and can be switched off. */
    @Test
    public void limitsShareOneRange() {
        for (ControlsField field : ControlsField.values()) {
            if (!field.canDisable()) {
                continue;
            }
            assertEquals(ControlsField.LIMIT_MIN, field.minimum(), EPS, field.key());
            assertEquals(ControlsField.LIMIT_MAX, field.maximum(), EPS, field.key());
        }
    }

    /**
     * Profiles written before limits could be disabled stored "off" as a large value. Those must
     * come back as disabled, not clamped to the new maximum, which would turn a dormant limit
     * into an active one.
     */
    @Test
    public void legacySentinelMigratesToDisabled() {
        String json = "{\"active\":\"a\",\"profiles\":{\"a\":"
            + "{\"skidLimit\":1000.0,\"forwardAccelLimit\":10.0}}}";
        ControlsConfig cfg = ControlsProfiles.fromJson(json).active();
        assertFalse(cfg.isEnabled(ControlsField.SKID_LIMIT));
        assertEquals(ControlsField.LIMIT_DISABLED_VALUE, cfg.skidLimit(), EPS);
        assertTrue(cfg.isEnabled(ControlsField.FORWARD_ACCEL_LIMIT));
        assertEquals(10.0, cfg.forwardAccelLimit(), EPS);
    }

    /** Enabled flags survive a JSON round trip. */
    @Test
    public void enabledFlagsRoundTrip() {
        ControlsProfiles profiles = new ControlsProfiles();
        profiles.putActive(profiles.active()
            .withEnabled(ControlsField.SKID_LIMIT, true)
            .with(ControlsField.SKID_LIMIT, 33.0)
            .withEnabled(ControlsField.FORWARD_ACCEL_LIMIT, false));
        ControlsConfig parsed = ControlsProfiles.fromJson(profiles.toJson()).active();
        assertTrue(parsed.isEnabled(ControlsField.SKID_LIMIT));
        assertEquals(33.0, parsed.skidLimit(), EPS);
        assertFalse(parsed.isEnabled(ControlsField.FORWARD_ACCEL_LIMIT));
        assertEquals(ControlsField.LIMIT_DISABLED_VALUE, parsed.forwardAccelLimit(), EPS);
    }

    /** Schemes default to the long-standing layout and survive a JSON round trip. */
    @Test
    public void schemeRoundTrips() {
        assertEquals(ControlScheme.STICK_TURN, ControlsConfig.defaults().scheme());

        ControlsProfiles profiles = new ControlsProfiles();
        profiles.putActive(profiles.active().withScheme(ControlScheme.TRIGGER_TURN));
        ControlsProfiles parsed = ControlsProfiles.fromJson(profiles.toJson());
        assertEquals(ControlScheme.TRIGGER_TURN, parsed.active().scheme());
    }

    /** A profile written before schemes existed loads as the long-standing layout. */
    @Test
    public void missingSchemeDefaults() {
        String json = "{\"active\":\"a\",\"profiles\":{\"a\":{\"translationMaxSpeed\":3.0}}}";
        assertEquals(ControlScheme.STICK_TURN, ControlsProfiles.fromJson(json).active().scheme());
    }

    /** An unrecognised scheme name falls back rather than throwing. */
    @Test
    public void unknownSchemeFallsBack() {
        assertEquals(ControlScheme.STICK_TURN, ControlScheme.fromName("nonsense"));
        assertEquals(ControlScheme.TRIGGER_TURN, ControlScheme.fromName("trigger_turn"));
        for (ControlScheme scheme : ControlScheme.values()) {
            assertEquals(scheme, ControlScheme.fromName(scheme.name()));
            assertFalse(scheme.label().isEmpty());
            assertFalse(scheme.summary().isEmpty());
        }
    }

    /**
     * Switching scheme must not disturb tuning. Drivers compare the two layouts back to back, so
     * a scheme change that silently re-tuned the robot would make that comparison meaningless.
     */
    @Test
    public void schemeDoesNotDisturbTuning() {
        ControlsConfig tuned = ControlsConfig.defaults()
            .with(ControlsField.TRANSLATION_MAX_SPEED, 3.25)
            .withEnabled(ControlsField.SKID_LIMIT, true)
            .withTranslationCurve(ControlsCurve.custom(new double[] {0, 0, 0.4, 0.15, 1, 1}));
        ControlsConfig switched = tuned.withScheme(ControlScheme.TRIGGER_TURN);

        assertEquals(ControlScheme.TRIGGER_TURN, switched.scheme());
        assertEquals(tuned.translationMaxSpeed(), switched.translationMaxSpeed(), EPS);
        assertEquals(tuned.skidLimit(), switched.skidLimit(), EPS);
        assertEquals(tuned.translationCurve(), switched.translationCurve());
        assertEquals(ControlScheme.STICK_TURN, tuned.scheme(), "original must be unchanged");
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
