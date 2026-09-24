package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

/** Tests for step-response metrics. */
public class StepResponseTest {

    /** A first-order rise reaches 90% at about 2.3 time constants. */
    @Test
    public void measuresRiseTimeOfFirstOrderResponse() {
        double tau = 0.2;
        StepResponse step = new StepResponse(0.0, 1.0, 2.0);
        for (int i = 0; i <= 2000; i++) {
            double t = i * 0.001;
            step.accept(t, 1.0 - Math.exp(-t / tau));
        }
        assertEquals(tau * Math.log(10), step.riseTime(), 0.01);
        assertEquals(0.0, step.overshoot(), 1e-6, "a first-order rise cannot overshoot");
        assertEquals(0.0, step.steadyStateError(), 1e-3);
    }

    /** Overshoot is reported as a fraction of the commanded step. */
    @Test
    public void measuresOvershoot() {
        StepResponse step = new StepResponse(0.0, 1.0, 1.0);
        step.accept(0.1, 0.5);
        step.accept(0.2, 1.2);
        step.accept(0.3, 1.0);
        for (double t = 0.8; t <= 1.0; t += 0.01) {
            step.accept(t, 1.0);
        }
        assertEquals(0.2, step.overshoot(), 1e-9);
    }

    /** A response that falls short reports the shortfall as steady-state error. */
    @Test
    public void measuresSteadyStateError() {
        // 1.7 of a 2.0 step is 85%, below the 90% rise threshold, so it never "rises".
        StepResponse step = new StepResponse(0.0, 2.0, 1.0);
        for (double t = 0.0; t <= 1.0; t += 0.01) {
            step.accept(t, 1.7);
        }
        assertEquals(-0.3, step.steadyStateError(), 1e-9);
        assertTrue(Double.isNaN(step.riseTime()), "never reached 90%, so rise time is undefined");
    }

    /** Metrics are undefined rather than wrong before any samples arrive. */
    @Test
    public void undefinedBeforeSamples() {
        StepResponse step = new StepResponse(0.0, 1.0, 1.0);
        assertTrue(Double.isNaN(step.riseTime()));
        assertTrue(Double.isNaN(step.steadyStateError()));
        assertEquals(0, step.sampleCount());
    }

    /** A zero-size step cannot divide by zero. */
    @Test
    public void zeroStepIsSafe() {
        StepResponse step = new StepResponse(1.0, 1.0, 1.0);
        step.accept(0.5, 1.0);
        assertEquals(0.0, step.overshoot(), 1e-9);
        assertTrue(Double.isFinite(step.overshoot()));
    }

    /** A downward step is measured the same way as an upward one. */
    @Test
    public void handlesDownwardSteps() {
        StepResponse step = new StepResponse(2.0, 0.0, 1.0);
        step.accept(0.1, 1.0);
        step.accept(0.2, 0.2);
        for (double t = 0.8; t <= 1.0; t += 0.01) {
            step.accept(t, 0.0);
        }
        assertEquals(0.2, step.riseTime(), 1e-9);
        assertEquals(0.0, step.steadyStateError(), 1e-9);
    }

}
