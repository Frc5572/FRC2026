package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GenerateEmptyIO;
import frc.robot.util.tunable.FlywheelConstants;


/**
 * Shooter IO Interface
 */
@GenerateEmptyIO
public interface ShooterIO {

    /** Shooter Inputs Class */
    @AutoLog
    public static class ShooterInputs {
        public AngularVelocity shooterAngularVelocity1 = RadiansPerSecond.zero();
        public AngularVelocity shooterAngularVelocity2 = RadiansPerSecond.zero();
        public Voltage shooterVoltage1 = Volts.zero();
        public Voltage shooterVoltage2 = Volts.zero();
        public Current shooterCurrent1 = Amps.zero();
        public Current shooterCurrent2 = Amps.zero();
    }

    public void updateInputs(ShooterInputs inputs);

    public void runDutyCycleVelocity(double velocity);

    public void runTorqueCurrentVelocity(double velocity);

    public void setConstants(FlywheelConstants constants);

}
