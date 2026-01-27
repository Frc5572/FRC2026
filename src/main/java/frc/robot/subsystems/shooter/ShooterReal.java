package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.Shooter;

/**
 * Shooter Real Implementation
 */
public final class ShooterReal implements ShooterIO {
    private TalonFX shooterMotor1 = new TalonFX(Shooter.motor1ID);
    private TalonFX shooterMotor2 = new TalonFX(Shooter.motor2ID);


    @Override
    public void setShootingSpeed(double speed) {
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shootingRPM1 = shooterMotor1.getVelocity().getValue();
        inputs.shootingRPM2 = shooterMotor1.getVelocity().getValue();
    }
}
