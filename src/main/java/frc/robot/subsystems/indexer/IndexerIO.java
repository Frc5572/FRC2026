package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs {
        double indexerFalconVelocity;
        double spinMotorVelocity;
        double indexerVortexVelocity;
    }

    public void updateInputs(IndexerIOInputs inputs);

    public void setIndexerSpeed(double speed);

    public void setSpinMotorSpeed(double speed);

    public static class Empty implements IndexerIO {

        @Override
        public void updateInputs(IndexerIOInputs inputs) {}

        @Override
        public void setIndexerSpeed(double speed) {}

        @Override
        public void setSpinMotorSpeed(double speed) {}

    }
}
