package frc.robot.tuning;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.util.GenerateEmptyIO;

/**
 * Source of live drivetrain tuning values and procedure requests.
 *
 * <p>
 * The live implementation talks to NetworkTables and the filesystem, both non-deterministic.
 * Routing everything through logged inputs means a replay runs the gains the robot actually had,
 * and re-runs the same procedure at the same moment.
 */
@GenerateEmptyIO
public interface TuningIO {

    /** Logged snapshot of the drivetrain tuning configuration and any pending request. */
    @AutoLog
    public class TuningInputs {
        /** Scalar values, indexed by {@link TuningField#ordinal()}. */
        public double[] values = TuningField.defaults();
        /** Module azimuth offsets in rotations, indexed by module. */
        public double[] moduleOffsets = new double[DrivetrainTuning.MODULE_COUNT];
        /** Name of the procedure the operator has asked for, or an empty string. */
        public String requestedProcedure = "";
        /** True when the in-memory configuration differs from what is on disk. */
        public boolean dirty = false;
    }

    /** Sample the current configuration and any pending procedure request. */
    public void updateInputs(TuningInputs inputs);

    /**
     * Record a value produced by a procedure, such as a fitted feedforward gain.
     *
     * @param field the field to set
     * @param value the value to store
     */
    public void reportValue(TuningField field, double value);

    /**
     * Record module azimuth offsets captured against the physical robot.
     *
     * @param offsets offsets in rotations, one per module
     */
    public void reportModuleOffsets(double[] offsets);

}
