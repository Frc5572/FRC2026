package frc.robot.tuning;

import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Owns the drivetrain tuning configuration and republishes it as logged inputs.
 *
 * <p>
 * A subsystem only so that {@link #periodic()} runs once per loop before any command; nothing
 * requires it. This is shop and pit equipment: the procedures it exposes move the robot, and are
 * bound to the tuner controller, which is not plugged in during a match.
 */
@NullMarked
public class Tuning extends SubsystemBase {

    private final TuningIO io;
    private final TuningInputsAutoLogged inputs = new TuningInputsAutoLogged();

    private DrivetrainTuning config = DrivetrainTuning.defaults();

    /**
     * Create the tuning subsystem.
     *
     * @param io the configuration source for the current run type
     */
    public Tuning(TuningIO io) {
        this.io = io;
        io.updateInputs(inputs);
        Logger.processInputs("Tuning", inputs);
        this.config = rebuild();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tuning", inputs);
        this.config = rebuild();
        Logger.recordOutput("Tuning/Dirty", inputs.dirty);
        Logger.recordOutput("Tuning/RequestedProcedure", inputs.requestedProcedure);
    }

    private DrivetrainTuning rebuild() {
        return DrivetrainTuning.of(inputs.values, inputs.moduleOffsets);
    }

    /** The tuning configuration in force for this loop. */
    public DrivetrainTuning config() {
        return config;
    }

    /**
     * A trigger that is true while the operator has asked for the named procedure.
     *
     * <p>
     * Because the request arrives through logged inputs, a replay fires the same trigger at the
     * same moment as the real run.
     *
     * @param name the procedure name, as the web page sends it
     * @return a trigger for binding the procedure's command
     */
    public Trigger requested(String name) {
        return new Trigger(() -> inputs.requestedProcedure.equals(name));
    }

    /**
     * Store a value produced by a procedure.
     *
     * @param field the field to set
     * @param value the value to store
     */
    public void report(TuningField field, double value) {
        io.reportValue(field, value);
    }

    /**
     * Store module azimuth offsets captured against the physical robot.
     *
     * @param offsets offsets in rotations, one per module
     */
    public void reportModuleOffsets(double[] offsets) {
        io.reportModuleOffsets(offsets);
    }

}
