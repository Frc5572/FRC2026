package frc.robot.subsystems.climber;

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
    private final TalonFX pivotMotor = new TalonFX(0);
    private final TalonFX telecopeMotorLeft = new TalonFX(0);
    private final TalonFX telescopeMotorRigh = new TalonFX(0);
    private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    private TalonFXConfiguration telescopeConfig = new TalonFXConfiguration();
    private StatusSignal<Angle> telescopePosition = telecopeMotorLeft.getPosition();
    private StatusSignal<Voltage> telescopeVoltage = telecopeMotorLeft.getMotorVoltage();
    private StatusSignal<AngularVelocity> telescopeVelocity = telecopeMotorLeft.getVelocity();
    private StatusSignal<Current> telescopeCurrent = telecopeMotorLeft.getStatorCurrent();
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public ClimberReal() {
        config();
    }

    private void config() {

        telescopeMotorRigh
            .setControl(new Follower(telecopeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));

        // right conf
        telescopeConfig.MotorOutput.NeutralMode = Constants.Climber.Telescope.BREAK;
        telescopeConfig.Feedback.SensorToMechanismRatio =
            Constants.Climber.Telescope.SensorToMechanismRatio;
        // telescopeConfig.CurrentLimits.StatorCurrentLimit = 150;
        // telescopeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // telescopeConfig.CurrentLimits.SupplyCurrentLimit = 100;
        // telescopeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // telescopeConfig.CurrentLimits.SupplyCurrentLowerTime = 5.0;
        // telescopeConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;


        // PID and feedforward

        // right
        telescopeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        telescopeConfig.Slot0.kP = Constants.Climber.Telescope.KP;
        telescopeConfig.Slot0.kI = Constants.Climber.Telescope.KI;
        telescopeConfig.Slot0.kD = Constants.Climber.Telescope.KD;
        telescopeConfig.Slot0.kS = Constants.Climber.Telescope.KS;
        telescopeConfig.Slot0.kV = Constants.Climber.Telescope.KV;
        telescopeConfig.Slot0.kA = Constants.Climber.Telescope.KA;
        telescopeConfig.Slot0.kG = Constants.Climber.Telescope.KG;
        telescopeConfig.MotionMagic.MotionMagicCruiseVelocity =
            Constants.Climber.Telescope.CVeleocity;
        telescopeConfig.MotionMagic.MotionMagicAcceleration =
            Constants.Climber.Telescope.Acceleration;
        telescopeConfig.MotionMagic.MotionMagicJerk = Constants.Climber.Telescope.Jerk;


        telecopeMotorLeft.getConfigurator().apply(telescopeConfig);
        telescopeMotorRigh.getConfigurator().apply(telescopeConfig);
    }

    public void setVoltage(double volts) {
        telecopeMotorLeft.setVoltage(volts);
    }

    public void setPower(double power) {
        telecopeMotorLeft.set(power);
    }

    public void setPositon(double position) {
        telecopeMotorLeft.setControl(m_request.withPosition(position));
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        BaseStatusSignal.refreshAll(telescopePosition, telescopeVelocity, telescopeVoltage,
            telescopeCurrent);
        inputs.position = Meters.of(telescopePosition.getValue().in(Rotations));
        inputs.velocity = telescopeVelocity.getValue();
        inputs.outputVoltage = telescopeVoltage.getValue();
        inputs.motorCurrent = telescopeCurrent.getValue();
    }
}
