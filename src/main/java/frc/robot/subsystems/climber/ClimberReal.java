package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ClimberReal implements ClimberIO {
    private final TalonFX pivotMotor = new TalonFX(Constants.Climber.Pivot.ID);
    private final TalonFX telecopeMotorLeft = new TalonFX(Constants.Climber.Telescope.LEFT_ID);
    private final TalonFX telescopeMotorRight = new TalonFX(Constants.Climber.Telescope.RIGHT_ID);
    private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    private TalonFXConfiguration telescopeConfig = new TalonFXConfiguration();
    private StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
    private StatusSignal<Voltage> pivotVoltage = pivotMotor.getMotorVoltage();
    private StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
    private StatusSignal<Current> pivotCurrent = pivotMotor.getStatorCurrent();
    private StatusSignal<Angle> telescopePosition = telecopeMotorLeft.getPosition();
    private StatusSignal<Voltage> telescopeVoltage = telecopeMotorLeft.getMotorVoltage();
    private StatusSignal<AngularVelocity> telescopeVelocity = telecopeMotorLeft.getVelocity();
    private StatusSignal<Current> telescopeCurrent = telecopeMotorLeft.getStatorCurrent();
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public ClimberReal() {
        config();
    }

    private void config() {
        // Telescope
        telescopeMotorRight
            .setControl(new Follower(telecopeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        telescopeConfig.MotorOutput.NeutralMode = Constants.Climber.Telescope.BREAK;
        telescopeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


        // Pivot
        pivotConfig.MotorOutput.NeutralMode = Constants.Climber.Pivot.BREAK;
        pivotConfig.Feedback.SensorToMechanismRatio =
            Constants.Climber.Pivot.SensorToMechanismRatio;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Slot0.kP = Constants.Climber.Pivot.KP;
        pivotConfig.Slot0.kI = Constants.Climber.Pivot.KI;
        pivotConfig.Slot0.kD = Constants.Climber.Pivot.KD;
        pivotConfig.Slot0.kS = Constants.Climber.Pivot.KS;
        pivotConfig.Slot0.kV = Constants.Climber.Pivot.KV;
        pivotConfig.Slot0.kA = Constants.Climber.Pivot.KA;
        pivotConfig.Slot0.kG = Constants.Climber.Pivot.KG;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.Pivot.CVeleocity;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.Pivot.Acceleration;
        pivotConfig.MotionMagic.MotionMagicJerk = Constants.Climber.Pivot.Jerk;


        pivotMotor.getConfigurator().apply(pivotConfig);
        telecopeMotorLeft.getConfigurator().apply(telescopeConfig);
        telescopeMotorRight.getConfigurator().apply(telescopeConfig);
    }

    public void setVoltageTelescope(double volts) {
        telecopeMotorLeft.setVoltage(volts);
    }

    public void setPowerTelescope(double power) {
        telecopeMotorLeft.set(power);
    }

    public void setVoltagePivot(double volts) {
        pivotMotor.setVoltage(volts);
    }

    public void setPowerPivot(double power) {
        pivotMotor.set(power);
    }

    public void setAnglePivot(Angle angle) {
        pivotMotor.setControl(m_request.withPosition(angle));
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        BaseStatusSignal.refreshAll(pivotPosition, pivotCurrent, pivotPosition, pivotVelocity,
            pivotVoltage, telescopeVelocity, telescopeVoltage, telescopeCurrent, telescopePosition);
        inputs.positionPivot = Degrees.of(pivotPosition.getValue().in(Rotations));
        inputs.velocityPivot = pivotVelocity.getValue();
        inputs.outputVoltagePivot = pivotVoltage.getValue();
        inputs.motorCurrentPivot = pivotCurrent.getValue();
        inputs.positionTelescope = Meters.of(telescopePosition.getValue().in(Rotations));
        inputs.velocityTelescope = telescopeVelocity.getValue();
        inputs.outputVoltageTelescope = telescopeVoltage.getValue();
        inputs.motorCurrentTelescope = telescopeCurrent.getValue();
    }
}
