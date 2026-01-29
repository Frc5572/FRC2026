package frc.robot.subsystems.adjustableHood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface AdjustableHoodIO {

    @AutoLog
    public static class AdjustableHoodInputs {
        public Angle relativeAngle = Rotations.of(0);
        public Voltage voltage = Volts.of(0);
        public Current current = Amps.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(0);
        public boolean atPosition;
    }

    public void setAdjustableHoodVoltage(Voltage volts);

    public void updateInputs(AdjustableHoodInputs inputs);

    public void setTargetAngle(Angle angle);
}
