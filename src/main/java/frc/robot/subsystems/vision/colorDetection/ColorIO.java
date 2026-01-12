package frc.robot.subsystems.vision.colorDetection;

import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;

public interface ColorIO {

    @AutoLog
    public class ColorInputs {
        Angle yaw = Radians.of(0.0);
        Boolean seesYellow = false;
    }

    public void updateInputs(ColorInputs inputs);
}
