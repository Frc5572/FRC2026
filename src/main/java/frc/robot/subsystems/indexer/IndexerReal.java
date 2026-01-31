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
    public SparkFlex magazine = new SparkFlex(Constants.Indexer.indexerID, MotorType.kBrushless);
    public TalonFX spindexer = new TalonFX(Constants.Indexer.spinMotorID);
    public RelativeEncoder encoder = magazine.getEncoder();
    private final StatusSignal<AngularVelocity> spinMotorVelocity = spindexer.getVelocity();
    public EncoderConfig magazineConfig = new EncoderConfig();
    private VelocityDutyCycle velocityDutyCycleRequest = new VelocityDutyCycle(0);

    public IndexerReal() {
        magazineConfig.velocityConversionFactor(1 / 60);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        BaseStatusSignal.refreshAll(spinMotorVelocity);
        inputs.spindexerVelocity = spinMotorVelocity.getValue();
        inputs.magazineVelocity =
            AngularVelocity.ofBaseUnits(encoder.getVelocity(), RotationsPerSecond);

    }

    @Override
    public void setSpindexerMotorDutyCycle(double dutyCycle) {
        spindexer.setControl(velocityDutyCycleRequest.withVelocity(dutyCycle));
    }

    @Override
    public void setMagazineDutyCycle(double dutyCycle) {
        magazine.set(dutyCycle);
    }
}
