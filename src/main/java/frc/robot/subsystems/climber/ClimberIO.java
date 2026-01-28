package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
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
        public Voltage outputVoltage;
        public Distance position;
        public AngularVelocity velocity;
        public Current motorCurrent;
    }

    public void updateInputs(ClimberInputs inputs);

    public void setVoltage(double volts);

    public void setPositon(double position);

    public default void setPower(double power) {}

}
