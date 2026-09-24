package frc.robot.tuning;

import org.jspecify.annotations.NullMarked;
import frc.robot.controls.ControlsField;

/**
 * Applies candidate acceleration limits on behalf of a tuning procedure.
 *
 * <p>
 * A ramp has to change both halves of a limit. Setting only the value is not enough: the skid and
 * tilt limits ship switched off, and a disabled limit reports the "no limit" sentinel whatever its
 * value, so a ramp that set values alone would drive around changing a number that never reached
 * the drivetrain.
 */
@NullMarked
public interface LimitApplier {

    /**
     * Set a limit's value.
     *
     * @param field the limit to change
     * @param value the candidate value
     */
    void setValue(ControlsField field, double value);

    /**
     * Switch a limit on or off.
     *
     * @param field the limit to change
     * @param enabled whether it should take effect
     */
    void setEnabled(ControlsField field, boolean enabled);

}
