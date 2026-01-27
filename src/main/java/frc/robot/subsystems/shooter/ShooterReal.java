package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.Shooter;

/**
 * Shooter Real Implementation
 */
public final class ShooterReal implements ShooterIO {
    private TalonFX shooterMotor1 = new TalonFX(Shooter.motor1ID);
    private TalonFX shooterMotor2 = new TalonFX(Shooter.motor2ID);
    private StatusSignal<AngularVelocity> shooterVelocity1 = shooterMotor1.getVelocity();
    private StatusSignal<AngularVelocity> shooterVelocity2 = shooterMotor2.getVelocity();


    @Override
    public void setShootingSpeed(double speed) {
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.shootingRPM1 = shooterVelocity1.getValue();
        inputs.shootingRPM2 = shooterVelocity2.getValue();
    }
}
