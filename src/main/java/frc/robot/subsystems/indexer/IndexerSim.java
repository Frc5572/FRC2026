package frc.robot.subsystems.indexer;

/**
 * Indexer simulation class
 */
public class IndexerSim implements IndexerIO {

    public boolean isFeeding = false;

    public int numFuel = 8;

    @Override
    public void updateInputs(IndexerInputs inputs) {}

    @Override
    public void setSpindexerMotorDutyCycle(double dutyCycle) {}

    @Override
    public void setMagazineDutyCycle(double dutyCycle) {
        isFeeding = dutyCycle > 0.1;
    }

    /** Increment fuel counter by 1 */
    public void addFuel() {
        numFuel++;
    }

}
