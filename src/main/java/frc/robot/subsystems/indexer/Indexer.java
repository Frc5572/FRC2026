package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer class
 */
public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    private void setMagazineDutyCycle(double dutyCycle) {
        io.setMagazineDutyCycle(dutyCycle);
        Logger.recordOutput("Indexer/MagazineDutyCycle", dutyCycle);
    }

    private void setSpindexerDutyCycle(double dutyCycle) {
        io.setSpindexerMotorDutyCycle(dutyCycle);
        Logger.recordOutput("Indexer/SpindexerDutyCycle", dutyCycle);
    }

    /**
     *
     * @param magazineDutyCycle power value from (-1) to 1
     * @param spindexerDutyCycle power value from (-1) to 1
     * @return command to set speed of indexer and spinner
     */
    public Command setSpeedCommand(double magazineDutyCycle, double spindexerDutyCycle) {
        return Commands
            .runEnd(() -> setSpindexerDutyCycle(spindexerDutyCycle), () -> setSpindexerDutyCycle(0))
            .alongWith(runEnd(() -> setMagazineDutyCycle(magazineDutyCycle),
                () -> setMagazineDutyCycle(0)));
    }
}
