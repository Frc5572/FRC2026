package frc.robot.subsystems.swerve.mod;

import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.jspecify.annotations.NullMarked;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.DeviceDebug;
import frc.robot.util.PhoenixSignals;

/** Real swerve module implementation (assumes two TalonFXs) */
@NullMarked
public class SwerveModuleReal implements SwerveModuleIO {

    // Drive Motor and Signals
    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveSupplyCurrentAmps;
    private final StatusSignal<Current> driveStatorCurrentAmps;

    // Angle Motor and Signals
    private final TalonFX angleMotor;
    private final TalonFXConfiguration angleConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> anglePosition;
    private final Queue<Double> anglePositionQueue;
    private final StatusSignal<AngularVelocity> angleVelocity;
    private final StatusSignal<Voltage> angleAppliedVolts;
    private final StatusSignal<Current> angleSupplyCurrentAmps;
    private final StatusSignal<Current> angleStatorCurrentAmps;

    // Absolute Encoder Signals
    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration absoluteConfig = new CANcoderConfiguration();
    private final StatusSignal<Angle> absolutePosition;

    // Absolute Offset
    private final Rotation2d angleOffset;

    /** Real swerve module implementation */
    public SwerveModuleReal(int index, PhoenixOdometryThread odometryThread) {
        boolean isCanivore = Constants.Swerve.isCanviore;

        CANBus loop = isCanivore ? new CANBus("*") : CANBus.roboRIO();
        driveMotor = new TalonFX(Constants.Swerve.modulesConstants[index].driveMotorId, loop);
        angleMotor = new TalonFX(Constants.Swerve.modulesConstants[index].angleMotorId, loop);
        absoluteEncoder = new CANcoder(Constants.Swerve.modulesConstants[index].canCoderId, loop);
        angleOffset = Constants.Swerve.modulesConstants[index].angleOffset;

        // Create drive motor signals
        drivePosition = driveMotor.getPosition();
        drivePositionQueue = odometryThread.registerSignal(driveMotor.getPosition().clone());
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrentAmps = driveMotor.getSupplyCurrent();
        driveStatorCurrentAmps = driveMotor.getStatorCurrent();

        // Create angle motor signals
        anglePosition = angleMotor.getPosition();
        anglePositionQueue = odometryThread.registerSignal(angleMotor.getPosition().clone());
        angleVelocity = angleMotor.getVelocity();
        angleAppliedVolts = angleMotor.getMotorVoltage();
        angleSupplyCurrentAmps = angleMotor.getSupplyCurrent();
        angleStatorCurrentAmps = angleMotor.getStatorCurrent();

        // Create absolute encoder signals
        absolutePosition = absoluteEncoder.getAbsolutePosition();

        configDriveMotor(index);
        configAngleMotor(index);
        configAngleEncoder(index);

        // Configure periodic frames
        PhoenixSignals.tryUntilOk(5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(Constants.Swerve.odometryFrequency,
                drivePosition, anglePosition));
        PhoenixSignals.tryUntilOk(5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts,
                driveSupplyCurrentAmps, driveStatorCurrentAmps, angleVelocity, angleAppliedVolts,
                angleSupplyCurrentAmps, angleStatorCurrentAmps));
        PhoenixSignals.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(driveMotor,
            angleMotor, absoluteEncoder));

        // Register signals for refresh
        PhoenixSignals.registerSignals(isCanivore, drivePosition, driveVelocity, driveAppliedVolts,
            driveSupplyCurrentAmps, driveStatorCurrentAmps, anglePosition, angleVelocity,
            angleAppliedVolts, angleSupplyCurrentAmps, angleStatorCurrentAmps, absolutePosition);

        DeviceDebug.register("SwerveModule_" + index + "_Drive", driveMotor);
        DeviceDebug.register("SwerveModule_" + index + "_Angle", angleMotor);
        DeviceDebug.register("SwerveModule_" + index + "_Encoder", absoluteEncoder);
    }

    private void configDriveMotor(int index) {
        /* Drive Motor Config */
        /* Motor Inverts and Neutral Mode */
        driveConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        driveConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        driveConfig.Feedback.SensorToMechanismRatio =
            Constants.Swerve.modulesConstants[index].kind.driveGearRatio;

        /* Current Limiting */
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.driveEnableCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentLowerLimit;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime =
            Constants.Swerve.driveCurrentLowerTimeThreshold;

        /* PID Config */
        Constants.Swerve.driveMotorPID.apply(driveConfig.Slot0);

        /* Open and Closed Loop Ramping */
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        PhoenixSignals.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
        PhoenixSignals.tryUntilOk(5, () -> driveMotor.getConfigurator().setPosition(0.0, 0.25));
    }

    private void configAngleMotor(int index) {
        /* Angle Motor Config */
        /* Motor Inverts and Neutral Mode */
        angleConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        angleConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        angleConfig.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfig.Feedback.SensorToMechanismRatio = 1.0;
        angleConfig.Feedback.RotorToSensorRatio =
            Constants.Swerve.modulesConstants[index].kind.angleGearRatio;
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        angleConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.angleEnableCurrentLimit;
        angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        angleConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.angleCurrentLowerLimit;
        angleConfig.CurrentLimits.SupplyCurrentLowerTime =
            Constants.Swerve.angleCurrentLowerTimeThreshold;

        /* PID Config */
        Constants.Swerve.angleMotorPID.apply(angleConfig.Slot0);

        PhoenixSignals.tryUntilOk(5, () -> angleMotor.getConfigurator().apply(angleConfig));
    }

    private void configAngleEncoder(int index) {
        /* Angle Encoder Config */
        absoluteConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        absoluteConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        absoluteConfig.MagnetSensor.MagnetOffset = -angleOffset.getRotations();

        PhoenixSignals.tryUntilOk(5,
            () -> absoluteEncoder.getConfigurator().apply(absoluteConfig, 0.25));
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        // Drive Motor inputs
        inputs.driveConnected = BaseStatusSignal.isAllGood(drivePosition, driveVelocity,
            driveAppliedVolts, driveSupplyCurrentAmps, driveStatorCurrentAmps);
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrentAmps.getValueAsDouble();
        inputs.driveStatorCurrentAmps = driveStatorCurrentAmps.getValueAsDouble();

        // Angle Motor inputs
        inputs.angleConnected = BaseStatusSignal.isAllGood(anglePosition, angleVelocity,
            angleAppliedVolts, angleSupplyCurrentAmps, angleStatorCurrentAmps);
        inputs.anglePosition = Rotation2d.fromRotations(anglePosition.getValueAsDouble());
        inputs.angleVelocityRadPerSec = Units.rotationsToRadians(angleVelocity.getValueAsDouble());
        inputs.angleAppliedVolts = angleAppliedVolts.getValueAsDouble();
        inputs.angleSupplyCurrentAmps = angleSupplyCurrentAmps.getValueAsDouble();
        inputs.angleStatorCurrentAmps = angleStatorCurrentAmps.getValueAsDouble();

        // Encoder inputs
        inputs.absoluteAngleConnected = BaseStatusSignal.isAllGood(absolutePosition);
        inputs.angleAbsolutePosition =
            Rotation2d.fromRotations(absolutePosition.getValueAsDouble());

        // Odometry inputs
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
        inputs.odometryAnglePositions =
            anglePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        anglePositionQueue.clear();
    }

    private final VoltageOut driveVoltage = new VoltageOut(0.0);

    @Override
    public void runDriveOpenLoop(double output) {
        driveMotor.setControl(driveVoltage.withOutput(output));
    }

    private final VoltageOut angleVoltage = new VoltageOut(0.0);

    @Override
    public void runAngleOpenLoop(double output) {
        angleMotor.setControl(angleVoltage.withOutput(output));
    }

    private final VelocityVoltage driveVelocityVoltage = new VelocityVoltage(0.0);

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveMotor.setControl(
            driveVelocityVoltage.withVelocity(velocityRadPerSec).withFeedForward(feedforward));
    }

    private final PositionVoltage anglePositionVoltage = new PositionVoltage(0.0);

    @Override
    public void runAnglePosition(Rotation2d rotation) {
        angleMotor.setControl(anglePositionVoltage.withPosition(rotation.getRotations()));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveConfig.Slot0.kS = kS;
        driveConfig.Slot0.kV = kV;
        driveConfig.Slot0.kA = kA;
        PhoenixSignals.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    }

    @Override
    public void setAnglePID(double kP, double kI, double kD) {
        angleConfig.Slot0.kP = kP;
        angleConfig.Slot0.kI = kI;
        angleConfig.Slot0.kD = kD;
        PhoenixSignals.tryUntilOk(5, () -> angleMotor.getConfigurator().apply(angleConfig, 0.25));
    }

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

    @Override
    public void setDriveBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            synchronized (driveConfig) {
                driveConfig.MotorOutput.NeutralMode =
                    enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                PhoenixSignals.tryUntilOk(5,
                    () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
            }
        });
    }

    @Override
    public void setAngleBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            synchronized (angleConfig) {
                angleConfig.MotorOutput.NeutralMode =
                    enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                PhoenixSignals.tryUntilOk(5,
                    () -> angleMotor.getConfigurator().apply(angleConfig, 0.25));
            }
        });
    }

}
