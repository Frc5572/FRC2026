package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.GenerateEmptyIO;


/**
 * Shooter IO Interface
 */
@GenerateEmptyIO
public interface ShooterIO {

    /** Shooter Inputs Class */
    @AutoLog
    public static class ShooterInputs {
        public AngularVelocity shooterAngularVelocity1;
        public AngularVelocity shooterAngularVelocity2;
        public Voltage shooterVoltage1;
        public Voltage shooterVoltage2;
        public Current shooterCurrent1;
        public Current shooterCurrent2;
    }

    public void updateInputs(ShooterInputs inputs);

    public void runShooterVelocity(double velocityRPM);

}
