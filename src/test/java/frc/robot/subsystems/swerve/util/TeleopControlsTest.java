package frc.robot.subsystems.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import frc.robot.controls.ControlsCurve;

/** Tests for driver input shaping. */
public class TeleopControlsTest {

    private static final double EPS = 1e-9;
    private static final ControlsCurve POWER = ControlsCurve.power();

    /** Input inside the deadband produces no output. */
    @Test
    public void deadbandSuppressesSmallInput() {
        assertEquals(0.0, TeleopControls.shapeMagnitude(0.05, 0.1, 0.95, POWER, 1.0), EPS);
        assertEquals(0.0, TeleopControls.shapeMagnitude(0.1, 0.1, 0.95, POWER, 1.0), EPS);
        assertTrue(TeleopControls.shapeMagnitude(0.11, 0.1, 0.95, POWER, 1.0) > 0.0);
    }

    /** Output rescales from zero at the deadband to one at the saturation point. */
    @Test
    public void rescalesBetweenDeadbandAndSaturation() {
        assertEquals(1.0, TeleopControls.shapeMagnitude(0.95, 0.1, 0.95, POWER, 1.0), EPS);
        assertEquals(1.0, TeleopControls.shapeMagnitude(1.0, 0.1, 0.95, POWER, 1.0), EPS);
        // Halfway between 0.1 and 0.95 is 0.525, which should give half output when linear.
        assertEquals(0.5, TeleopControls.shapeMagnitude(0.525, 0.1, 0.95, POWER, 1.0), 1e-6);
    }

    /** A higher exponent softens the low end without changing the endpoints. */
    @Test
    public void exponentShapesOnlyTheMiddle() {
        double linear = TeleopControls.shapeMagnitude(0.525, 0.1, 0.95, POWER, 1.0);
        double squared = TeleopControls.shapeMagnitude(0.525, 0.1, 0.95, POWER, 2.0);
        assertTrue(squared < linear, "squared response should be softer at mid stick");
        assertEquals(0.0, TeleopControls.shapeMagnitude(0.1, 0.1, 0.95, POWER, 2.0), EPS);
        assertEquals(1.0, TeleopControls.shapeMagnitude(0.95, 0.1, 0.95, POWER, 2.0), EPS);
    }

    /** Output is never negative and never exceeds one, whatever the inputs. */
    @Test
    public void outputStaysInUnitRange() {
        for (double m = 0.0; m <= 1.5; m += 0.01) {
            for (double exp : new double[] {1.0, 1.5, 2.0, 3.0}) {
                double out = TeleopControls.shapeMagnitude(m, 0.1, 0.95, POWER, exp);
                assertTrue(out >= 0.0 && out <= 1.0, "out of range at magnitude " + m);
            }
        }
    }

    /** A deadband at or above the saturation point must not divide by zero. */
    @Test
    public void degenerateRangeIsSafe() {
        double out = TeleopControls.shapeMagnitude(1.0, 0.9, 0.5, POWER, 1.0);
        assertTrue(Double.isFinite(out) && out >= 0.0 && out <= 1.0);
    }

    /** A zero time constant reproduces the previous hard stop. */
    @Test
    public void zeroTimeConstantStopsImmediately() {
        assertEquals(0.0, TeleopControls.coastFactor(0.0, 0.02), EPS);
    }

    /**
     * The coast decays exponentially, matching the linear drag MoSim applies when the stick is
     * released. One time constant should leave roughly 1/e of the speed.
     */
    @Test
    public void coastDecaysExponentially() {
        double tau = 0.33;
        assertEquals(Math.exp(-1.0), TeleopControls.coastFactor(tau, tau), 1e-9);
        double factor = TeleopControls.coastFactor(tau, 0.02);
        assertTrue(factor > 0.9 && factor < 1.0, "a single 20 ms loop should barely decay");

        // Stepping at the loop rate must agree with the closed form over the time actually
        // elapsed; 0.33 s is not a whole number of 20 ms steps, so compare against the sum.
        double v = 1.0;
        double elapsed = 0.0;
        for (int i = 0; i < 17; i++) {
            v *= TeleopControls.coastFactor(tau, 0.02);
            elapsed += 0.02;
        }
        assertEquals(Math.exp(-elapsed / tau), v, 1e-9);
    }

}
