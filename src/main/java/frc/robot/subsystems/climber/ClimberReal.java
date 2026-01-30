package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Real hardware implementation of the climber subsystem.
 *
 * <p>
 * This class interfaces with actual TalonFX motor controllers for both the telescope extension and
 * pivot rotation mechanisms. It configures motors with Phoenix 6 features including motion magic
 * control for the pivot and voltage control for the telescope.
 *
 * @see ClimberIO
 */
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

    /**
     * Constructs a new ClimberReal instance and configures all motors and sensors.
     */
    public ClimberReal() {
        config();
    }

    /**
     * Configures all TalonFX motors with their respective settings.
     *
     * <p>
     * Configures the telescope motors as followers with opposed alignment, and the pivot motor with
     * motion magic control parameters including PID gains, feedforward constants, and motion
     * constraints.
     */
    private void config() {
        // Telescope
        telescopeMotorRight
            .setControl(new Follower(telecopeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        telescopeConfig.MotorOutput.NeutralMode = Constants.Climber.Telescope.BREAK;
        telescopeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        telescopeConfig.Feedback.SensorToMechanismRatio =
            Constants.Climber.Telescope.SENSOR_TO_MECHANISM_RATIO;

        // Pivot
        pivotConfig.MotorOutput.NeutralMode = Constants.Climber.Pivot.BREAK;
        pivotConfig.Feedback.SensorToMechanismRatio =
            Constants.Climber.Pivot.SENSOR_TO_MECHANISM_RATIO;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Slot0.kP = Constants.Climber.Pivot.KP;
        pivotConfig.Slot0.kI = Constants.Climber.Pivot.KI;
        pivotConfig.Slot0.kD = Constants.Climber.Pivot.KD;
        pivotConfig.Slot0.kS = Constants.Climber.Pivot.KS;
        pivotConfig.Slot0.kV = Constants.Climber.Pivot.KV;
        pivotConfig.Slot0.kA = Constants.Climber.Pivot.KA;
        pivotConfig.Slot0.kG = Constants.Climber.Pivot.KG;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.Pivot.C_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.Pivot.ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = Constants.Climber.Pivot.JERK;

        pivotMotor.getConfigurator().apply(pivotConfig);
        telecopeMotorLeft.getConfigurator().apply(telescopeConfig);
        telescopeMotorRight.getConfigurator().apply(telescopeConfig);
    }

    /**
     * Sets the voltage for the telescope motor.
     *
     * @param volts the voltage to apply to the telescope motor
     */
    @Override
    public void setVoltageTelescope(double volts) {
        telecopeMotorLeft.setVoltage(volts);
    }

    /**
     * Sets the power output for the telescope motor.
     *
     * @param power the power output, typically in the range [-1.0, 1.0]
     */
    @Override
    public void setPowerTelescope(double power) {
        telecopeMotorLeft.set(power);
    }

    /**
     * Sets the voltage for the pivot motor.
     *
     * @param volts the voltage to apply to the pivot motor
     */
    @Override
    public void setVoltagePivot(double volts) {
        pivotMotor.setVoltage(volts);
    }

    /**
     * Sets the power output for the pivot motor.
     *
     * @param power the power output, typically in the range [-1.0, 1.0]
     */
    @Override
    public void setPowerPivot(double power) {
        pivotMotor.set(power);
    }

    /**
     * Sets the target angle for the pivot mechanism using motion magic control.
     *
     * @param angle the desired pivot angle in any angle unit
     */
    @Override
    public void setAnglePivot(Angle angle) {
        pivotMotor.setControl(m_request.withPosition(angle));
    }

    /**
     * Updates the input container with current sensor readings from all motors.
     *
     * <p>
     * Refreshes all status signals and updates the input object with current positions, velocities,
     * voltages, and currents for both mechanisms.
     *
     * @param inputs the input container to populate with current hardware state
     */
    @Override
    public void updateInputs(ClimberInputs inputs) {
        BaseStatusSignal.refreshAll(pivotPosition, pivotCurrent, pivotPosition, pivotVelocity,
            pivotVoltage, telescopeVelocity, telescopeVoltage, telescopeCurrent, telescopePosition);
        inputs.positionPivot = pivotPosition.getValue();
        inputs.velocityPivot = pivotVelocity.getValue();
        inputs.outputVoltagePivot = pivotVoltage.getValue();
        inputs.motorCurrentPivot = pivotCurrent.getValue();
        inputs.positionTelescope = Meters.of(telescopePosition.getValue().in(Rotations));
        inputs.velocityTelescope = telescopeVelocity.getValue();
        inputs.outputVoltageTelescope = telescopeVoltage.getValue();
        inputs.motorCurrentTelescope = telescopeCurrent.getValue();
    }
}
