package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.PhoenixSignals;
import frc.robot.util.tunable.FlywheelConstants;

/**
 * real implementation of indexer
 */
public class IndexerReal implements IndexerIO {
    public TalonFX magazine = new TalonFX(Constants.Indexer.indexerID);
    public TalonFX spindexer = new TalonFX(Constants.Indexer.spinMotorID);
    private final StatusSignal<AngularVelocity> magazineStatusVelocity = magazine.getVelocity();
    private final StatusSignal<AngularVelocity> spinMotorVelocity = spindexer.getVelocity();
    public TalonFXConfiguration magazineConfig = new TalonFXConfiguration();
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    private TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    private double desiredSpeed = 3.0;
    private double magazineDesiredSpeed = 3.0;


    /** Real Indexer Implementation */
    public IndexerReal() {
        spindexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        magazineConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        setSpindexerConstants(Constants.Indexer.spindexerConstants);
        setMagazineConstants(Constants.Indexer.magazineConstants);
        PhoenixSignals.registerSignals(false, spinMotorVelocity);
        PhoenixSignals.registerSignals(false, magazineStatusVelocity);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        inputs.spindexerVelocity = spinMotorVelocity.getValue();
        inputs.magazineVelocity = magazineStatusVelocity.getValue();
    }

    @Override
    public void setSpindexerMotorDutyCycle(double dutyCycle) {
        if (Math.abs(dutyCycle) < 0.01) {
            spindexer.setControl(dutyCycleOut.withOutput(dutyCycle));
        } else {
            spindexer.setControl(velocityVoltage.withVelocity(dutyCycle * desiredSpeed));
        }
    }

    @Override
    public void setMagazineDutyCycle(double dutyCycle) {
        if (Math.abs(dutyCycle) < 0.01) {
            magazine.setControl(dutyCycleOut.withOutput(dutyCycle));
        } else {
            magazine.setControl(velocityVoltage.withVelocity(dutyCycle * magazineDesiredSpeed));
        }
    }

    @Override
    public void setSpindexerConstants(FlywheelConstants constants) {
        constants.pid.apply(spindexerConfig.Slot0);
        spindexer.getConfigurator().apply(spindexerConfig);
        desiredSpeed = constants.maxDutyCycle;
    }

    @Override
    public void setMagazineConstants(FlywheelConstants constants) {
        constants.pid.apply(magazineConfig.Slot0);
        magazine.getConfigurator().apply(magazineConfig);
        magazineDesiredSpeed = constants.maxDutyCycle;
    }
}
