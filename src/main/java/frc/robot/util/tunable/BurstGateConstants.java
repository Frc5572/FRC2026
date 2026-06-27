package frc.robot.util.tunable;

/** Tunables for the shooter burst gate and run-hot flywheel setpoint. */
@Tunable
public class BurstGateConstants {

    /**
     * How far from the nominal flywheel speed toward the max of the shot window to run, as a 0->1
     * fraction. 0 keeps the old nominal setpoint; 1 runs at the top of the window. Running hot
     * leaves more sag headroom so a continuous burst stays in-window.
     */
    public double setpointFraction = 0.7;

    /**
     * How far below the run-hot setpoint (as a 0->1 fraction of the shot window width) the flywheel
     * must recover before re-arming a burst. The upper edge of the hysteresis band that prevents
     * per-shot indexer flip-flopping.
     */
    public double armFraction = 0.1;

    /** Create new burst gate constants. */
    public BurstGateConstants() {

    }

}
