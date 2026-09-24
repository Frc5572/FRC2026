package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import java.util.Arrays;
import org.junit.jupiter.api.Test;
import frc.robot.controls.ControlsField;

/** Tests that the ramp specs encode the procedures from software-sessions.md. */
public class LimitRampSpecTest {

    /** "Set to a logical value (around 10m/s^2), increase until..." (doc lines 162-164). */
    @Test
    public void forwardStartsAtTenAndRampsUp() {
        LimitRampSpec spec = LimitRampSpec.forward(() -> 0.0);
        assertEquals(ControlsField.FORWARD_ACCEL_LIMIT, spec.field());
        assertEquals(10.0, spec.start(), 1e-9);
        assertTrue(spec.step() > 0, "the doc says increase");
        assertTrue(spec.trippedWhenAbove(), "failure is the follow error growing");
    }

    /** "Increase until robot starts tipping" (doc lines 169-171), stopping well before it does. */
    @Test
    public void tiltRampsUpAndStopsWellBeforeTipping() {
        LimitRampSpec spec = LimitRampSpec.tilt(() -> 0.0);
        assertEquals(ControlsField.FORWARD_TILT_LIMIT, spec.field());
        assertTrue(spec.step() > 0, "the doc says increase");
        assertTrue(spec.trippedWhenAbove());
        assertTrue(spec.threshold() <= 6.0,
            "a tipping robot is not a safe stop condition; a few degrees of lift is");
    }

    /** "Decrease until robot stops skidding" (doc line 176). */
    @Test
    public void skidRampsDownAndStopsWhenClean() {
        LimitRampSpec spec = LimitRampSpec.skid(() -> 0.0);
        assertEquals(ControlsField.SKID_LIMIT, spec.field());
        assertTrue(spec.step() < 0, "the doc says decrease");
        assertTrue(!spec.trippedWhenAbove(),
            "ramping down, the ramp ends when the detector comes back clean");
        assertTrue(spec.threshold() > 1.0, "a ratio of 1.0 means no skid at all");
    }

    /** Every ramp starts inside its field's range and opens the limits it is not testing. */
    @Test
    public void specsAreSelfConsistent() {
        for (LimitRampSpec spec : Arrays.asList(LimitRampSpec.forward(() -> 0.0),
            LimitRampSpec.tilt(() -> 0.0), LimitRampSpec.skid(() -> 0.0))) {
            assertTrue(spec.start() >= spec.field().minimum()
                && spec.start() <= spec.field().maximum(),
                spec.field().key() + " starts outside its own range");
            assertTrue(spec.burstSeconds() > 0 && spec.settleSeconds() > 0);
            for (ControlsField other : spec.othersToOpen()) {
                assertTrue(other != spec.field(),
                    "the limit under test must not be opened along with the others");
                assertTrue(other.canDisable(), other.key() + " is not an acceleration limit");
            }
        }
    }

}
