package frc.robot.subsystems.indexer;

/**
 * Indexer simulation class
 */
public class IndexerSim implements IndexerIO {

    public boolean isFeeding = false;

    @Override
    public void updateInputs(IndexerInputs inputs) {}

    @Override
    public void setSpindexerMotorDutyCycle(double dutyCycle) {}

    @Override
    public void setMagazineDutyCycle(double dutyCycle) {
        isFeeding = dutyCycle > 0.1;
    }

}
