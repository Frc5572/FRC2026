package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixSignals;
import frc.robot.util.tunable.PIDConstants;

/** turret hardware */
public class TurretReal implements TurretIO {
    private TalonFX turretMotor = new TalonFX(Constants.Turret.TurretMotorID);
    private TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    private CANcoder turretCANcoder2 = new CANcoder(Constants.Turret.TurretCANcoderID2);
    private CANcoderConfiguration canCoder1Config = new CANcoderConfiguration();
    private CANcoderConfiguration canCoder2Config = new CANcoderConfiguration();

    private StatusSignal<Angle> turretPosition = turretMotor.getPosition();
    private StatusSignal<Voltage> turretVoltage = turretMotor.getMotorVoltage();
    private StatusSignal<Current> turretCurrent = turretMotor.getStatorCurrent();
    private StatusSignal<AngularVelocity> turretVelocity = turretMotor.getVelocity();
    private StatusSignal<Angle> canCoder2Pos = turretCANcoder2.getPosition();
    public final PositionVoltage mmVoltage = new PositionVoltage(0);
    private final VoltageOut voltage = new VoltageOut(0.0);

    /** Real Turret Implementation */
    public TurretReal() {
        // PID and feedforward

        Constants.Turret.pid.apply(turretConfig.Slot0);

        canCoder1Config.MagnetSensor.SensorDirection = Constants.Turret.canCoder1Invert;
        canCoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
            Constants.Turret.turretCANCoderDiscontinuity;
        canCoder2Config.MagnetSensor.SensorDirection = Constants.Turret.canCoder2Invert;
        canCoder2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
            Constants.Turret.turretCANCoderDiscontinuity;

        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turretConfig.Feedback.FeedbackRemoteSensorID = turretCANcoder2.getDeviceID();
        turretConfig.Feedback.SensorToMechanismRatio = Constants.Turret.gear2Gearing;
        turretConfig.Feedback.RotorToSensorRatio =
            Constants.Turret.motorGearing / Constants.Turret.gear2Gearing;

        turretMotor.getConfigurator().apply(turretConfig);
        turretCANcoder2.getConfigurator().apply(canCoder2Config);

        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        resetPosition(Degree.of(0));

        BaseStatusSignal.setUpdateFrequencyForAll(50, turretPosition, turretVoltage, turretCurrent,
            canCoder2Pos);
        PhoenixSignals.registerSignals(false, turretPosition, turretVoltage, turretCurrent,
            canCoder2Pos);
        ParentDevice.optimizeBusUtilizationForAll(turretCANcoder2, turretMotor);
    }

    @Override
    public void setTurretVoltage(Voltage volts) {
        turretMotor.setControl(voltage.withOutput(volts));
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.gear2AbsoluteAngle = canCoder2Pos.getValue();

        inputs.relativeAngle = turretPosition.getValue().unaryMinus().in(Rotations);
        inputs.voltage = turretVoltage.getValue();
        inputs.current = turretCurrent.getValue();
        inputs.velocity = turretVelocity.getValue();

        inputs.positionValue = turretPosition.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Angle angle, AngularVelocity velocity) {
        turretMotor.setControl(
            mmVoltage.withPosition(angle.unaryMinus()).withVelocity(velocity.unaryMinus()));
    }

    @Override
    public void resetPosition(Angle angle) {
        turretMotor.setPosition(angle);
        turretCANcoder2.setPosition(angle);
    }


    @Override
    public void setPID(PIDConstants constants) {
        constants.apply(turretConfig.Slot0);
        turretMotor.getConfigurator().apply(turretConfig);
    }
}
