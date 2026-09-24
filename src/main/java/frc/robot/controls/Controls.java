package frc.robot.controls;

import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tuning.LimitApplier;

/**
 * Owns the driver-tunable control configuration and republishes it as logged inputs.
 *
 * <p>
 * This is a subsystem only so that {@link #periodic()} is called once per loop before any command
 * runs; nothing ever requires it. Consumers take {@link #config()} &mdash; or, more usually, a
 * {@code Supplier<ControlsConfig>} bound to it &mdash; rather than reading static constants, so
 * the values in force are explicit at each call site.
 *
 * <p>
 * In replay the IO is the generated empty implementation and every value is restored from the
 * log, which keeps a replay faithful to the configuration the robot actually drove with.
 */
@NullMarked
public class Controls extends SubsystemBase implements LimitApplier {

    private final ControlsIO io;
    private final ControlsInputsAutoLogged inputs = new ControlsInputsAutoLogged();

    private ControlsConfig config = ControlsConfig.defaults();

    /**
     * Create the controls subsystem.
     *
     * @param io the configuration source for the current run type
     */
    public Controls(ControlsIO io) {
        this.io = io;
        io.updateInputs(inputs);
        Logger.processInputs("Controls", inputs);
        this.config = rebuild();
    }

    /** Reassemble the configuration from the logged inputs, so replay matches the real run. */
    private ControlsConfig rebuild() {
        return ControlsConfig.of(inputs.values, inputs.enabled,
            ControlsCurve.of(inputs.translationCurveMode, inputs.translationCurveKnots),
            ControlsCurve.of(inputs.rotationCurveMode, inputs.rotationCurveKnots))
            .withScheme(ControlScheme.fromName(inputs.scheme));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Controls", inputs);
        this.config = rebuild();
        Logger.recordOutput("Controls/ActiveProfile", inputs.activeProfile);
        Logger.recordOutput("Controls/Scheme", inputs.scheme);
        Logger.recordOutput("Controls/Dirty", inputs.dirty);
    }

    /** The configuration in force for this loop. */
    public ControlsConfig config() {
        return config;
    }

    /** The name of the profile currently selected. */
    public String activeProfile() {
        return inputs.activeProfile;
    }

    /**
     * Set one value from the robot side.
     *
     * <p>
     * Used by the acceleration-limit procedures to apply a candidate limit so its effect can be
     * measured. Goes through the IO, so the change is logged like any other.
     *
     * @param field the field to set
     * @param value the value to store
     */
    @Override
    public void setValue(ControlsField field, double value) {
        io.requestValue(field, value);
    }

    @Override
    public void setEnabled(ControlsField field, boolean enabled) {
        io.requestEnabled(field, enabled);
    }

    /** True when there are live edits that have not been written to disk. */
    public boolean isDirty() {
        return inputs.dirty;
    }

}
