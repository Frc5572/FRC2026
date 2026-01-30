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
        /** Output voltage applied to the telescope motor, in volts. */
        public Voltage outputVoltageTelescope = Volts.zero();

        /** Angular velocity of the telescope motor, in rotations per second. */
        public AngularVelocity velocityTelescope = RotationsPerSecond.zero();

        /** Motor current draw from the telescope motor, in amps. */
        public Current motorCurrentTelescope = Amps.zero();

        /** Angular velocity of the pivot motor, in rotations per second. */
        public AngularVelocity velocityPivot = RotationsPerSecond.zero();

        /** Motor current draw from the pivot motor, in amps. */
        public Current motorCurrentPivot = Amps.zero();

        /** Output voltage applied to the pivot motor, in volts. */
        public Voltage outputVoltagePivot = Volts.zero();

        /** Current angle of the pivot mechanism, in degrees. */
        public Angle positionPivot = Degrees.zero();

        /** Current extension of the telescope mechanism, in meters. */
        public Distance positionTelescope = Meters.zero();
    }

    /**
     * Updates the input container with current climber sensor data.
     *
     * @param inputs the input container to populate with current hardware state
     */
    public void updateInputs(ClimberInputs inputs);

    /**
     * Sets the voltage for the telescope motor.
     *
     * @param volts the voltage to apply to the telescope motor
     */
    public void setVoltageTelescope(double volts);

    /**
     * Sets the power output for the telescope motor.
     *
     * @param power the power output, typically in the range [-1.0, 1.0]
     */
    public void setPowerTelescope(double power);

    /**
     * Sets the voltage for the pivot motor.
     *
     * @param volts the voltage to apply to the pivot motor
     */
    public void setVoltagePivot(double volts);

    /**
     * Sets the power output for the pivot motor.
     *
     * @param power the power output, typically in the range [-1.0, 1.0]
     */
    public void setPowerPivot(double power);

    /**
     * Sets the target angle for the pivot mechanism.
     *
     * @param angle the desired pivot angle in any angle unit
     */
    public void setAnglePivot(Angle angle);
}
