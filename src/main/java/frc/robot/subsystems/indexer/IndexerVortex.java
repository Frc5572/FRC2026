package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class IndexerVortex implements IndexerIO {
    public SparkFlex indexer = new SparkFlex(Constants.Indexer.indexerID, MotorType.kBrushless);
    public TalonFX spinMotor = new TalonFX(Constants.Indexer.spinMotorID);
    public RelativeEncoder encoder = indexer.getEncoder();

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spinMotorVelocity = spinMotor.getVelocity().getValueAsDouble();
        inputs.indexerVortexVelocity = encoder.getVelocity();

    }

    @Override
    public void setIndexerSpeed(double speed) {
        indexer.set(speed);
    }

    @Override
    public void setSpinMotorSpeed(double speed) {
        spinMotor.set(speed);
    }
}
