package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.GenerateEmptyIO;

/**
 * Shooter IO Interface
 */
@GenerateEmptyIO
public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public AngularVelocity shootingRPM1;
        public AngularVelocity shootingRPM2;
    }

    public void updateInputs(ShooterIOInputs inputs);

    public void setShootingSpeed(double speed);

}
