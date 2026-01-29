package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private CANcoderConfiguration canCoder1Config = new CANcoderConfiguration();
    private CANcoderConfiguration canCoder2Config = new CANcoderConfiguration();

    private StatusSignal<Angle> turretPosition = turretMotor.getPosition();
    private StatusSignal<Voltage> turretVoltage = turretMotor.getMotorVoltage();
    private StatusSignal<Current> turretCurrent = turretMotor.getStatorCurrent();
    private StatusSignal<AngularVelocity> turretVelocity = turretMotor.getVelocity();
    private StatusSignal<Angle> canCoder1Pos = turretCANcoder1.getAbsolutePosition();
    private StatusSignal<Angle> canCoder2Pos = turretCANcoder2.getAbsolutePosition();

    public final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);
    private final VoltageOut voltage = new VoltageOut(0.0);

    /** Real Turret Implementation */
    public TurretReal() {
        configTurret();

        turretMotor.setNeutralMode(NeutralModeValue.Brake);


        BaseStatusSignal.setUpdateFrequencyForAll(50, turretPosition, turretVoltage, turretCurrent,
            canCoder1Pos, canCoder2Pos);

    }

    private void configTurret() {

        turretConfig.Feedback.SensorToMechanismRatio = Constants.Turret.motorGearing;

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

        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            Constants.Turret.maxAngle.in(Rotations);
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            Constants.Turret.minAngle.in(Rotations);

        canCoder1Config.MagnetSensor.SensorDirection = Constants.Turret.canCoder1Invert;
        canCoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
            Constants.Turret.turretCANCoderDiscontinuity;
        canCoder2Config.MagnetSensor.SensorDirection = Constants.Turret.canCoder2Invert;
        canCoder2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
            Constants.Turret.turretCANCoderDiscontinuity;

        turretMotor.getConfigurator().apply(turretConfig);
        turretCANcoder1.getConfigurator().apply(canCoder1Config);
        turretCANcoder2.getConfigurator().apply(canCoder2Config);
    }

    @Override
    public void setTurretVoltage(Voltage volts) {
        turretMotor.setControl(voltage.withOutput(volts));
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        BaseStatusSignal.refreshAll(turretPosition, turretVoltage, turretCurrent, turretVelocity,
            canCoder1Pos, canCoder2Pos);

        inputs.gear1AbsoluteAngle = new Rotation2d(canCoder1Pos.getValue());
        inputs.gear2AbsoluteAngle = new Rotation2d(canCoder2Pos.getValue());

        inputs.relativeAngle = turretPosition.getValue();
        inputs.voltage = turretVoltage.getValue();
        inputs.current = turretCurrent.getValue();
        inputs.velocity = turretVelocity.getValue();

        inputs.positionValue = turretPosition.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Angle angle) {
        turretMotor.setControl(mmVoltage.withPosition(angle));
    }

    @Override
    public void resetPosition(Angle angle) {
        turretMotor.setPosition(angle);
    }
}
