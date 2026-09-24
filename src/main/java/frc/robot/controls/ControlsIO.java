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
        /** True when the in-memory profiles differ from what is on disk. */
        public boolean dirty = false;
    }

    /** Sample the current configuration. */
    public void updateInputs(ControlsInputs inputs);

}
