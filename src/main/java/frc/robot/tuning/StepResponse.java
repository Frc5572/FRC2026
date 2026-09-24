package frc.robot.tuning;

import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;

/**
 * Metrics for one half-cycle of a step-response test.
 *
 * <p>
 * 1690's tuning method (software-sessions.md, lines 62&ndash;70) is to zero the feedback gains,
 * drive the mechanism between setpoints, fit the feedforward model until the error is small, and
 * only then add a little kP &mdash; judging progress from a graph of setpoint against measured.
 * This class turns that graph into three numbers so the judgement is not purely by eye.
 *
 * <p>
 * Accumulate samples with {@link #accept}, then read the metrics. All three are undefined until at
 * least one sample has arrived.
 */
@NullMarked
public final class StepResponse {

    /** Fraction of the step considered "risen", matching the usual 90% rise-time convention. */
    private static final double RISE_FRACTION = 0.9;

    /** Window at the end of the step over which steady-state error is averaged, in seconds. */
    private static final double SETTLE_WINDOW = 0.25;

    private final double start;
    private final double target;
    private final double duration;

    private double riseTime = Double.NaN;
    private double peak;
    private double settleSum;
    private int settleCount;
    private int samples;

    /**
     * Begin measuring a step.
     *
     * @param start the measured value at the moment the step was commanded
     * @param target the value being stepped to
     * @param duration how long the step is held, in seconds
     */
    public StepResponse(double start, double target, double duration) {
        this.start = start;
        this.target = target;
        this.duration = duration;
        this.peak = start;
    }

    /**
     * Add a sample.
     *
     * @param elapsed seconds since the step was commanded
     * @param measured the measured value
     */
    public void accept(double elapsed, double measured) {
        samples++;
        double span = target - start;
        if (Math.abs(span) > 1e-9) {
            double progress = (measured - start) / span;
            if (Double.isNaN(riseTime) && progress >= RISE_FRACTION) {
                riseTime = elapsed;
            }
        }
        if (Math.abs(measured - start) > Math.abs(peak - start)) {
            peak = measured;
        }
        if (elapsed >= duration - SETTLE_WINDOW) {
            settleSum += measured;
            settleCount++;
        }
    }

    /** Seconds to first reach 90% of the commanded step, or NaN if it never did. */
    public double riseTime() {
        return riseTime;
    }

    /**
     * Overshoot beyond the target as a fraction of the step size.
     *
     * @return 0.0 when the response never passed the target
     */
    public double overshoot() {
        double span = target - start;
        if (Math.abs(span) < 1e-9) {
            return 0.0;
        }
        double over = (peak - target) / span;
        return Math.max(0.0, over);
    }

    /** Mean error over the last {@value #SETTLE_WINDOW} seconds of the step. */
    public double steadyStateError() {
        if (settleCount == 0) {
            return Double.NaN;
        }
        return settleSum / settleCount - target;
    }

    /** Number of samples accumulated. */
    public int sampleCount() {
        return samples;
    }

    /**
     * Publish the metrics under the given log key.
     *
     * @param key log key prefix, for example {@code "Tuning/DriveStep"}
     */
    public void log(String key) {
        Logger.recordOutput(key + "/Target", target);
        Logger.recordOutput(key + "/RiseTime", riseTime);
        Logger.recordOutput(key + "/Overshoot", overshoot());
        Logger.recordOutput(key + "/SteadyStateError", steadyStateError());
        Logger.recordOutput(key + "/Samples", samples);
    }

}
