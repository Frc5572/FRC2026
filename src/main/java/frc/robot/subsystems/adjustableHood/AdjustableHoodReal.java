package frc.robot.subsystems.adjustableHood;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class AdjustableHoodReal implements AdjustableHoodIO {

    private final TalonFX hoodMotor = new TalonFX(Constants.AdjustableHood.HoodMotorID);
    private TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    private CANcoder hoodCANcoder = new CANcoder(Constants.AdjustableHood.HoodCANCoderID);
    private CANcoderConfiguration hoodCANcoderConfig = new CANcoderConfiguration();

    private StatusSignal<Angle> hoodAngle = hoodMotor.getPosition();
    private StatusSignal<Voltage> hoodVoltage = hoodMotor.getMotorVoltage();
    private StatusSignal<Current> hoodCurrent = hoodMotor.getStatorCurrent();
    private StatusSignal<AngularVelocity> hoodVelocity = hoodMotor.getVelocity();
    private StatusSignal<Angle> hoodCANcoderAngle = hoodCANcoder.getPosition();

    public AdjustableHoodReal() {
        configAdjustableHood();

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        BaseStatusSignal.setUpdateFrequencyForAll(50, hoodAngle, hoodVoltage, hoodCurrent,
            hoodCANcoderAngle);
    }

    private void configAdjustableHood() {

        // PID and feedforward

        hoodConfig.Slot0.kP = Constants.AdjustableHood.KP;
        hoodConfig.Slot0.kI = Constants.AdjustableHood.KI;
        hoodConfig.Slot0.kD = Constants.AdjustableHood.KD;
        hoodConfig.Slot0.kS = Constants.AdjustableHood.KS;
        hoodConfig.Slot0.kV = Constants.AdjustableHood.KV;
        hoodConfig.Slot0.kA = Constants.AdjustableHood.KA;
        hoodConfig.Slot0.kG = Constants.AdjustableHood.KG;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.AdjustableHood.MMCVelocity;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.AdjustableHood.MMAcceleration;
        hoodConfig.MotionMagic.MotionMagicJerk = Constants.AdjustableHood.MMJerk;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            Constants.AdjustableHood.maxAngle.in(Rotations);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            Constants.AdjustableHood.minAngle.in(Rotations);

        hoodCANcoderConfig.MagnetSensor.SensorDirection =
            Constants.AdjustableHood.hoodCANCoderInvert;
        hoodCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
            Constants.AdjustableHood.hoodCANcoderDiscontinuity;

        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodCANcoder.getConfigurator().apply(hoodCANcoderConfig);
    }

    private final VoltageOut voltage = new VoltageOut(0.0);

    @Override
    public void setAdjustableHoodVoltage(Voltage volts) {
        hoodMotor.setControl(voltage.withOutput(volts));
    }

    private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

    @Override
    public void updateInputs(AdjustableHoodInputs inputs) {
        BaseStatusSignal.refreshAll(hoodAngle, hoodVoltage, hoodCurrent, hoodVelocity,
            hoodCANcoderAngle);

        inputs.relativeAngle = hoodAngle.getValue();
        inputs.voltage = hoodVoltage.getValue();
        inputs.current = hoodCurrent.getValue();
        inputs.velocity = hoodVelocity.getValue();

        inputs.hoodLocation = hoodAngle.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Angle angle) {
        hoodMotor.setControl(mmVoltage.withPosition(angle));
    }

}
