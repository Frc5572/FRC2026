package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GenerateEmptyIO;

/**
 * Elevator IO Class for Climber
 */

@GenerateEmptyIO
public interface ClimberIO {

    /**
     * Inputs Class for Climber
     */

    @AutoLog
    public class ClimberInputs {
        public boolean atPositon;
        public Voltage outputVoltageTelescope;
        public AngularVelocity velocityTelescope;
        public Current motorCurrentTelescope;
        public AngularVelocity velocityPivot;
        public Current motorCurrentPivot;
        public Voltage outputVoltagePivot;
        public Angle positionPivot;
    }

    public void updateInputs(ClimberInputs inputs);

    public void setVoltageTelescope(double volts);

    public void setPowerTelescope(double power);

    public void setVoltagePivot(double volts);

    public void setPowerPivot(double power);

    public void setAnglePivot(Angle angle);
}
