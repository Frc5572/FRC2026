package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** turret hardware */
public class TurretReal implements TurretIO {
    private TalonFX turretMotor = new TalonFX(Constants.Turret.TurretMotorID);
    private TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    private CANcoder turretCANcoder1 = new CANcoder(Constants.Turret.TurretCANcoderID2);
    private CANcoder turretCANcoder2 = new CANcoder(Constants.Turret.TurretCANcoderID1);
    private CANcoderConfiguration CANcoder1Config = new CANcoderConfiguration();
    private CANcoderConfiguration CANcoder2Config = new CANcoderConfiguration();

    private StatusSignal<Angle> turretPosition = turretMotor.getPosition();
    private StatusSignal<Voltage> turretVoltage = turretMotor.getMotorVoltage();
    private StatusSignal<Current> turretCurrent = turretMotor.getStatorCurrent();
    private StatusSignal<AngularVelocity> turretVelocity = turretMotor.getVelocity();

    public TurretReal() {
        configTurret();

        BaseStatusSignal.setUpdateFrequencyForAll(50, turretPosition, turretVoltage, turretCurrent);

    }

    private void configTurret() {
        turretMotor.getConfigurator().apply(turretConfig);
        turretCANcoder1.getConfigurator().apply(CANcoder1Config);
        turretCANcoder2.getConfigurator().apply(CANcoder2Config);

        // PID and feedforward

        turretConfig.Slot0.kP = Constants.Turret.KP;
        turretConfig.Slot0.kI = Constants.Turret.KI;
        turretConfig.Slot0.kD = Constants.Turret.KD;
        turretConfig.Slot0.kS = Constants.Turret.KS;
        turretConfig.Slot0.kV = Constants.Turret.KV;
        turretConfig.Slot0.kA = Constants.Turret.KA;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Turret.MMCVelocity;
        turretConfig.MotionMagic.MotionMagicAcceleration = Constants.Turret.MMAcceleration;
        turretConfig.MotionMagic.MotionMagicJerk = Constants.Turret.MMJerk;
    }

    private final VoltageOut voltage = new VoltageOut(0.0);

    @Override
    public void setTurretVoltage(Voltage volts) {
        turretMotor.setControl(voltage.withOutput(volts));
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        BaseStatusSignal.refreshAll(turretPosition, turretVoltage, turretCurrent, turretVelocity,
            turretCANcoder1.getAbsolutePosition(), turretCANcoder2.getAbsolutePosition());

        inputs.gear1AbsoluteAngle =
            new Rotation2d(turretCANcoder1.getAbsolutePosition().getValue());
        inputs.gear2AbsoluteAngle =
            new Rotation2d(turretCANcoder2.getAbsolutePosition().getValue());

        inputs.relativeAngle = turretPosition.getValue();
        inputs.voltage = turretVoltage.getValue();
        inputs.current = turretCurrent.getValue();
        inputs.velocity = turretVelocity.getValue();
    }

    private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

    @Override
    public void setTargetAngle(Angle angle) {
        turretMotor.setControl(mmVoltage.withPosition(angle));
    }

    @Override
    public void resetPosition(Angle angle) {
        TurretIO.super.resetPosition(angle);
    }
}
