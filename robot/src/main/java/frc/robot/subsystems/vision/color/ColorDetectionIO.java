package frc.robot.subsystems.vision.color;

import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;

/** Color detector IO */
public interface ColorDetectionIO {

    /** Color detection inputs */
    @AutoLog
    public class ColorInputs {
        Angle yaw = Radians.of(0.0);
        boolean seesYellow = false;
        String error = "";
    }

    public void updateInputs(ColorInputs inputs);

    /** Empty IO layer for replay */
    public class Empty implements ColorDetectionIO {

        @Override
        public void updateInputs(ColorInputs inputs) {
            /* Intentionally empty */
        }
    }
}
