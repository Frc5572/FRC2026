package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

/**
 * real implementation of indexer
 */
public class IndexerReal implements IndexerIO {
    public SparkFlex magazine;
    public TalonFX spindexer = new TalonFX(Constants.Indexer.spinMotorID);
    public RelativeEncoder encoder;
    private final StatusSignal<AngularVelocity> spinMotorVelocity = spindexer.getVelocity();
    public EncoderConfig magazineConfig;
    private VelocityDutyCycle velocityDutyCycleRequest = new VelocityDutyCycle(0);

    private boolean magazineConnected = false;

    /** Real Indexer Implementation */
    public IndexerReal() {
        try {
            magazine = new SparkFlex(Constants.Indexer.indexerID, MotorType.kBrushless);

            if (magazine.getFirmwareVersion() == 0) {
                throw new Exception("Motor not found");
            }
            magazineConfig = new EncoderConfig();
            magazineConfig.velocityConversionFactor(1.0 / 60.0);
            encoder = magazine.getEncoder();
            magazineConnected = true;
        } catch (Exception e) {
            System.out.println("magazine initialization failed: " + e.getMessage());
        }
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        BaseStatusSignal.refreshAll(spinMotorVelocity);
        inputs.spindexerVelocity = spinMotorVelocity.getValue();

        inputs.magazineMotorConnected = magazineConnected;

        if (magazineConnected && magazine != null) {
            inputs.magazineVelocity = RotationsPerSecond.of(encoder.getVelocity());
        } else {
            inputs.magazineVelocity = RotationsPerSecond.of(0);
        }
    }

    @Override
    public void setSpindexerMotorDutyCycle(double dutyCycle) {
        spindexer.setControl(velocityDutyCycleRequest.withVelocity(dutyCycle));
    }

    @Override
    public void setMagazineDutyCycle(double dutyCycle) {
        if (magazineConnected && magazine != null) {
            magazine.set(dutyCycle);
        }
    }
}
