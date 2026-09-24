package frc.robot.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

/** Tests for the driver response curve. */
public class ControlsCurveTest {

    private static final double EPS = 1e-9;

    /** Power mode reproduces the linear and squared presets exactly. */
    @Test
    public void powerModeMatchesPresets() {
        ControlsCurve curve = ControlsCurve.power();
        assertEquals(0.5, curve.evaluate(0.5, 1.0), EPS);
        assertEquals(0.25, curve.evaluate(0.5, 2.0), EPS);
        assertEquals(0.0, curve.evaluate(0.0, 2.0), EPS);
        assertEquals(1.0, curve.evaluate(1.0, 2.0), EPS);
    }

    /** A two-knot custom curve is the identity line. */
    @Test
    public void defaultCustomCurveIsLinear() {
        ControlsCurve curve = ControlsCurve.custom(ControlsCurve.defaultKnots());
        for (double x = 0.0; x <= 1.0; x += 0.05) {
            assertEquals(x, curve.evaluate(x, 1.0), 1e-9);
        }
    }

    /** Knots are interpolated through, not merely approached. */
    @Test
    public void passesThroughItsKnots() {
        ControlsCurve curve =
            ControlsCurve.custom(new double[] {0, 0, 0.25, 0.1, 0.75, 0.8, 1, 1});
        assertEquals(0.1, curve.evaluate(0.25, 1.0), 1e-9);
        assertEquals(0.8, curve.evaluate(0.75, 1.0), 1e-9);
    }

    /**
     * The whole point of monotone interpolation: a curve with an aggressive knot must never dip
     * or run backwards, because on a drivetrain that feels like the robot fighting the stick.
     */
    @Test
    public void neverDecreases() {
        double[][] shapes = {
            {0, 0, 0.1, 0.9, 1, 1},          // very aggressive early rise
            {0, 0, 0.9, 0.05, 1, 1},         // very flat then a cliff
            {0, 0, 0.2, 0.05, 0.4, 0.5, 0.6, 0.55, 1, 1},
            {0, 0, 0.5, 0.5, 1, 1},
        };
        for (double[] shape : shapes) {
            ControlsCurve curve = ControlsCurve.custom(shape);
            double previous = -1e-9;
            for (double x = 0.0; x <= 1.0; x += 0.002) {
                double y = curve.evaluate(x, 1.0);
                assertTrue(y >= previous - 1e-9,
                    "curve decreased at x=" + x + " (" + y + " < " + previous + ")");
                assertTrue(y >= 0.0 && y <= 1.0, "curve left [0,1] at x=" + x);
                previous = y;
            }
        }
    }

    /** Centred stick must stop and buried stick must command full output, whatever the knots. */
    @Test
    public void endpointsArePinned() {
        ControlsCurve curve = ControlsCurve.custom(new double[] {0.3, 0.4, 0.6, 0.2, 0.9, 0.7});
        assertEquals(0.0, curve.evaluate(0.0, 1.0), EPS);
        assertEquals(1.0, curve.evaluate(1.0, 1.0), EPS);
    }

    /** Out-of-order, duplicated and out-of-range knots are repaired rather than rejected. */
    @Test
    public void sanitizesHostileInput() {
        double[] knots = ControlsCurve.sanitize(
            new double[] {0.8, 2.0, 0.2, -1.0, 0.8001, 0.5, 0.5, 0.5});
        assertTrue(knots.length >= 4);
        for (int i = 0; i + 2 < knots.length; i += 2) {
            assertTrue(knots[i] < knots[i + 2], "knots must be strictly increasing in x");
        }
        for (double v : knots) {
            assertTrue(v >= 0.0 && v <= 1.0, "knot outside the unit square");
        }
        assertEquals(0.0, knots[0], EPS);
        assertEquals(1.0, knots[knots.length - 2], EPS);
    }

    /** Malformed input degrades to a straight line instead of an undrivable curve. */
    @Test
    public void malformedKnotsFallBackToLinear() {
        assertTrue(java.util.Arrays.equals(ControlsCurve.defaultKnots(),
            ControlsCurve.sanitize(new double[] {0.5})));
        assertTrue(java.util.Arrays.equals(ControlsCurve.defaultKnots(),
            ControlsCurve.sanitize(new double[] {0, 0, Double.NaN, 1})));
        assertTrue(java.util.Arrays.equals(ControlsCurve.defaultKnots(),
            ControlsCurve.sanitize(new double[] {0, 0, 1})));
    }

    /** More than MAX_KNOTS knots are truncated rather than accepted. */
    @Test
    public void capsKnotCount() {
        double[] many = new double[2 * (ControlsCurve.MAX_KNOTS + 6)];
        for (int i = 0; i < many.length / 2; i++) {
            many[2 * i] = i / (double) (many.length / 2 - 1);
            many[2 * i + 1] = many[2 * i];
        }
        assertTrue(ControlsCurve.custom(many).knotCount() <= ControlsCurve.MAX_KNOTS);
    }

    /** Toggling modes keeps the knots, so switching to a preset and back loses no work. */
    @Test
    public void modeToggleKeepsKnots() {
        ControlsCurve custom =
            ControlsCurve.custom(new double[] {0, 0, 0.4, 0.15, 1, 1});
        ControlsCurve asPower = custom.withMode(ControlsCurve.Mode.POWER);
        assertEquals(ControlsCurve.Mode.POWER, asPower.mode());
        assertTrue(java.util.Arrays.equals(custom.knots(), asPower.knots()));
        assertEquals(custom, asPower.withMode(ControlsCurve.Mode.CUSTOM));
    }

    /** An unrecognised mode name falls back to the power preset. */
    @Test
    public void unknownModeFallsBackToPower() {
        assertEquals(ControlsCurve.Mode.POWER,
            ControlsCurve.of("nonsense", ControlsCurve.defaultKnots()).mode());
        assertEquals(ControlsCurve.Mode.CUSTOM,
            ControlsCurve.of("custom", ControlsCurve.defaultKnots()).mode());
    }

    /** Curves survive the JSON round trip that profiles use. */
    @Test
    public void curvesRoundTripThroughJson() {
        ControlsProfiles profiles = new ControlsProfiles();
        profiles.putActive(profiles.active().withTranslationCurve(
            ControlsCurve.custom(new double[] {0, 0, 0.35, 0.12, 1, 1})));
        ControlsProfiles parsed = ControlsProfiles.fromJson(profiles.toJson());
        ControlsCurve curve = parsed.active().translationCurve();
        assertEquals(ControlsCurve.Mode.CUSTOM, curve.mode());
        assertEquals(3, curve.knotCount());
        assertEquals(0.12, curve.evaluate(0.35, 1.0), 1e-9);
    }

}
