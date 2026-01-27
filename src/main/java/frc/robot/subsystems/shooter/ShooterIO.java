package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.GenerateEmptyIO;


/**
 * Shooter IO Interface
 */
@GenerateEmptyIO
public interface ShooterIO {

    /** Shooter IO Inputs Class */
    @AutoLog
    public static class ShooterInputs {
        public AngularVelocity shootingRPM1;
        public AngularVelocity shootingRPM2;
    }

    public void updateInputs(ShooterInputs inputs);

    public void setShootingSpeed(double speed);

}
