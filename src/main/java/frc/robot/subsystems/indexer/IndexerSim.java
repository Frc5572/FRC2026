package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.tunable.FlywheelConstants;

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

    @Override
    public void setConstants(FlywheelConstants constants) {}

    @Override
    public void setVoltageSpindexer(Voltage volts) {}

}
