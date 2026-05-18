package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.GenerateEmptyIO;

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
    }

    public void updateInputs(ShooterInputs inputs);

    public void runVolts(double volts);

    public void runVelocity(double velocity);

    // public void setConstants(FlywheelConstants constants);

}
