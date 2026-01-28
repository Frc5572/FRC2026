package frc.robot.subsystems.adjustableHood;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface AdjustableHoodIO {

    @AutoLog
    public static class AdjustableHoodInputs {
        public Angle relativeAngle;
        public Voltage voltage;
        public Current current;
        public AngularVelocity velocity;
    }

    public void setAdjustableHoodVoltage(Voltage volts);

    public void updateInputs(AdjustableHoodInputs inputs);

    public void setTargetAngle(Angle angle);
}
