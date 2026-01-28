package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Shooter;

/** Shooter Real Implementation */
public final class ShooterReal implements ShooterIO {
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private TalonFXConfiguration motor1Config = new TalonFXConfiguration();
    private TalonFXConfiguration motor2Config = new TalonFXConfiguration();

    private StatusSignal<AngularVelocity> shooterVelocity1;
    private StatusSignal<AngularVelocity> shooterVelocity2;
    private StatusSignal<Voltage> shooterVoltage1;
    private StatusSignal<Voltage> shooterVoltage2;
    private StatusSignal<Current> shooterCurrent1;
    private StatusSignal<Current> shooterCurrent2;

    /** Shooter Real Implementation Constructor */
    public ShooterReal() {
        shooterMotor1 = new TalonFX(Shooter.motor1ID);
        shooterMotor2 = new TalonFX(Shooter.motor2ID);
        shooterVelocity1 = shooterMotor1.getVelocity();
        shooterVelocity2 = shooterMotor2.getVelocity();
        shooterVoltage1 = shooterMotor1.getMotorVoltage();
        shooterVoltage2 = shooterMotor2.getMotorVoltage();
        shooterCurrent1 = shooterMotor1.getSupplyCurrent();
        shooterCurrent2 = shooterMotor2.getSupplyCurrent();

        configMotors();
    }

    private void configMotors() {
        motor1Config.MotorOutput.Inverted = Shooter.shooterMotor1Invert;
        motor1Config.MotorOutput.NeutralMode = Shooter.shooterNeutralMode;
        motor2Config.MotorOutput.Inverted = Shooter.shooterMotor2Invert;
        motor2Config.MotorOutput.NeutralMode = Shooter.shooterNeutralMode;

        motor1Config.Slot0.kP = Shooter.shooterKP;
        motor1Config.Slot0.kI = Shooter.shooterKI;
        motor1Config.Slot0.kD = Shooter.shooterKD;
        motor1Config.Slot0.kS = Shooter.shooterKS;
        motor1Config.Slot0.kV = Shooter.shooterKV;

        motor2Config.Slot0.kP = Shooter.shooterKP;
        motor2Config.Slot0.kI = Shooter.shooterKI;
        motor2Config.Slot0.kD = Shooter.shooterKD;
        motor2Config.Slot0.kS = Shooter.shooterKS;
        motor2Config.Slot0.kV = Shooter.shooterKV;

        shooterMotor1.getConfigurator().apply(motor1Config);
        shooterMotor2.getConfigurator().apply(motor2Config);
    }

    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0.0);

    @Override
    public void runShooterVelocity(double velocity) {
        shooterMotor1
            .setControl(shooterVelocityVoltage.withVelocity(velocity).withFeedForward(0.1));
        shooterMotor2
            .setControl(shooterVelocityVoltage.withVelocity(velocity).withFeedForward(0.1));
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        BaseStatusSignal.refreshAll(shooterVelocity1, shooterVelocity2, shooterVoltage1,
            shooterVoltage2, shooterCurrent1, shooterCurrent2);
        inputs.shooterAngularVelocity1 = shooterVelocity1.getValue();
        inputs.shooterAngularVelocity2 = shooterVelocity2.getValue();
        inputs.shooterVoltage1 = shooterVoltage1.getValue();
        inputs.shooterVoltage2 = shooterVoltage2.getValue();
        inputs.shooterCurrent1 = shooterCurrent1.getValue();
        inputs.shooterCurrent2 = shooterCurrent2.getValue();
    }
}
