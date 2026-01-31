package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer class
 */
public class Indexer extends SubsystemBase {
    IndexerIO io;
    IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    private void setIndexerDutyCycle(double dutyCycle) {
        io.setIndexerDutyCycle(dutyCycle);
        Logger.recordOutput("Indexer/IndexerDutyCycle", dutyCycle);
    }

    private void setSpinMotorDutyCycle(double dutyCycle) {
        io.setSpinMotorDutyCycle(dutyCycle);
        Logger.recordOutput("Indexer/SpinMotorDutyCycle", dutyCycle);
    }

    /**
     * 
     * @param IndexerDutyCycle power value from (-1) to 1
     * @param SpinMotorDutyCycle power value from (-1) to 1
     * @return command to set speed of indexer and spinner
     */
    public Command setSpeedCommand(double IndexerDutyCycle, double SpinMotorDutyCycle) {
        return Commands
            .runEnd(() -> setSpinMotorDutyCycle(SpinMotorDutyCycle), () -> setSpinMotorDutyCycle(0),
                this)
            .alongWith(
                runEnd(() -> setIndexerDutyCycle(IndexerDutyCycle), () -> setIndexerDutyCycle(0)));
    }
}
