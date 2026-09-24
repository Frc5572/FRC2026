package frc.robot.controls;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.util.GenerateEmptyIO;

/**
 * Source of driver-tunable control values.
 *
 * <p>
 * The live implementation talks to NetworkTables and the filesystem, both of which are
 * non-deterministic. Routing the values through logged inputs means a replay reproduces the exact
 * configuration the robot was driving with, rather than whatever is checked in today.
 */
@GenerateEmptyIO
public interface ControlsIO {

    /**
     * Logged snapshot of the active control configuration.
     *
     * <p>
     * {@code values} is indexed by {@link ControlsField#ordinal()}. Logging the array rather than
     * one field per value keeps this class stable as fields are added.
     */
    @AutoLog
    public class ControlsInputs {
        /** Name of the profile these values came from. */
        public String activeProfile = ControlsProfiles.DEFAULT_PROFILE;
        /** Values of the active profile, indexed by {@link ControlsField#ordinal()}. */
        public double[] values = ControlsField.defaults();
        /** Which set of driver bindings is in force, as a {@link ControlScheme} name. */
        public String scheme = ControlScheme.defaultScheme().name();
        /** Which fields take effect, indexed by {@link ControlsField#ordinal()}. */
        public boolean[] enabled = ControlsField.defaultEnabledFlags();
        /** Translation response curve mode, as a {@link ControlsCurve.Mode} name. */
        public String translationCurveMode = ControlsCurve.Mode.POWER.name();
        /** Translation curve knots, as {@code [x0, y0, x1, y1, ...]}. */
        public double[] translationCurveKnots = ControlsCurve.defaultKnots();
        /** Rotation response curve mode, as a {@link ControlsCurve.Mode} name. */
        public String rotationCurveMode = ControlsCurve.Mode.POWER.name();
        /** Rotation curve knots, as {@code [x0, y0, x1, y1, ...]}. */
        public double[] rotationCurveKnots = ControlsCurve.defaultKnots();
        /** True when the in-memory profiles differ from what is on disk. */
        public boolean dirty = false;
    }

    /** Sample the current configuration. */
    public void updateInputs(ControlsInputs inputs);

}
