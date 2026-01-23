package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IndexerFalcon implements IndexerIO {
    public TalonFX indexer = new TalonFX(Constants.Indexer.indexerID);
    public TalonFX spinMotor = new TalonFX(Constants.Indexer.spinMotorID);

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spinMotorVelocity = spinMotor.getVelocity().getValueAsDouble();
        inputs.indexerVortexVelocity = indexer.getVelocity().getValueAsDouble();

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
