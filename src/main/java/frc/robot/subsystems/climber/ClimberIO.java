package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GenerateEmptyIO;

/**
 * Hardware abstraction interface for the climber subsystem.
 *
 * <p>
 * This interface defines the contract for all climber hardware implementations, enabling easy
 * switching between real hardware and simulation. It handles both the telescope extension and pivot
 * rotation mechanisms.
 */
@GenerateEmptyIO
public interface ClimberIO {

    /**
     * Input container for climber sensor and state data.
     *
     * <p>
     * These fields are automatically logged and contain all relevant information from the climber
     * hardware including motor voltages, velocities, currents, and mechanism positions.
     */
    @AutoLog
    public class ClimberInputs {
        public Voltage outputVoltageTelescope = Volts.zero();
        public AngularVelocity velocityTelescope = RotationsPerSecond.zero();
        public Current motorCurrentTelescope = Amps.zero();
        public AngularVelocity velocityPivot = RotationsPerSecond.zero();
        public Current motorCurrentPivot = Amps.zero();
        public Voltage outputVoltagePivot = Volts.zero();
        public Angle positionPivot = Degrees.zero();
        public Distance positionTelescope = Meters.zero();
    }

    public void updateInputs(ClimberInputs inputs);

    public void setVoltageTelescope(double volts);

    public void setPowerTelescope(double power);

    public void setVoltagePivot(double volts);

    public void setPowerPivot(double power);

    public void setAnglePivot(Angle angle);
}
