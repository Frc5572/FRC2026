package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.util.PhoenixSignals;
import frc.robot.util.tunable.FlywheelConstants;

/** Shooter Real Implementation */
public final class ShooterReal implements ShooterIO {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<AngularVelocity> shooterVelocity1;
    private final StatusSignal<AngularVelocity> shooterVelocity2;
    private final StatusSignal<Voltage> shooterVoltage1;
    private final StatusSignal<Voltage> shooterVoltage2;
    private final StatusSignal<Current> shooterCurrent1;
    private final StatusSignal<Current> shooterCurrent2;

    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
        new VelocityTorqueCurrentFOC(0.0);

    public ShooterReal() {
        shooterMotor1 = new TalonFX(Shooter.motor1ID);
        shooterMotor2 = new TalonFX(Shooter.motor2ID);
        motorConfig = new TalonFXConfiguration();

        shooterVelocity1 = shooterMotor1.getVelocity();
        shooterVelocity2 = shooterMotor2.getVelocity();
        shooterVoltage1 = shooterMotor1.getMotorVoltage();
        shooterVoltage2 = shooterMotor2.getMotorVoltage();
        shooterCurrent1 = shooterMotor1.getStatorCurrent();
        shooterCurrent2 = shooterMotor2.getStatorCurrent();

        setConstants(Constants.Shooter.constants);

        PhoenixSignals.registerSignals(false, shooterVelocity1, shooterVelocity2, shooterVoltage1,
            shooterVoltage2, shooterCurrent1, shooterCurrent2);
    }

    @Override
    public void runDutyCycleVelocity(double velocity) {
        shooterMotor1.setControl(velocityDutyCycle.withVelocity(velocity));
    }

    @Override
    public void runTorqueCurrentVelocity(double velocity) {
        shooterMotor1.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.shooterAngularVelocity1 = shooterVelocity1.getValue();
        inputs.shooterAngularVelocity2 = shooterVelocity2.getValue();
        inputs.shooterVoltage1 = shooterVoltage1.getValue();
        inputs.shooterVoltage2 = shooterVoltage2.getValue();
        inputs.shooterCurrent1 = shooterCurrent1.getValue();
        inputs.shooterCurrent2 = shooterCurrent2.getValue();
    }

    @Override
    public void setConstants(FlywheelConstants constants) {
        motorConfig.MotorOutput.Inverted = constants.isReversed ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.Slot0.kP = 9999.0;
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.holdCurrent;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        motorConfig.MotorOutput.PeakForwardDutyCycle = constants.maxDutyCycle;
        motorConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

        shooterMotor1.getConfigurator().apply(motorConfig);
        shooterMotor2.getConfigurator().apply(motorConfig);

        shooterMotor2
            .setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }
}
