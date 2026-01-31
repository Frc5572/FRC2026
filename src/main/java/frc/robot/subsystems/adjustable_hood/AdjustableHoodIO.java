package frc.robot.subsystems.adjustable_hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GenerateEmptyIO;

/**
 * Hardware abstraction layer for the adjustable hood subsystem.
 */
@GenerateEmptyIO
public interface AdjustableHoodIO {

    /**
     * Container for all adjustable hood sensor inputs.
     */
    @AutoLog
    public static class AdjustableHoodInputs {
        public Angle relativeAngle = Rotations.of(0);
        public Voltage voltage = Volts.of(0);
        public Current current = Amps.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(0);
        public double hoodLocation = 0.0;
    }

    public void setAdjustableHoodVoltage(Voltage volts);

    /**
     * Updates the provided {@link AdjustableHoodInputs} structure with the latest sensor values.
     * 
     * @param inputs updates inputs
     */
    public void updateInputs(AdjustableHoodInputs inputs);

    /**
     * Commands the adjustable hood to move to a specific angle
     * 
     * @param angle - the wanted angle
     */
    public void setTargetAngle(Angle angle);
}
