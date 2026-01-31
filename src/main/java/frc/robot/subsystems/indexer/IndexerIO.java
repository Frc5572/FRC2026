package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO
public interface IndexerIO {
    @AutoLog
    public class IndexerInputs {
        double indexerFalconVelocity;
        double spinMotorVelocity;
        double indexerVortexVelocity;
    }

    public void updateInputs(IndexerInputs inputs);

    public void setIndexerDutyCycle(double dutyCycle);

    public void setSpinMotorDutyCycle(double dutyCycle);

}
