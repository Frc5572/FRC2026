package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private void setIndexerSpeed(double speed) {
        io.setIndexerDutyCycle(speed);
        Logger.recordOutput("Indexer/IndexerSpeed", speed);
    }

    private void setSpinMotorSpeed(double dutyCycle) {
        io.setSpinMotorDutyCycle(dutyCycle);
        Logger.recordOutput("Indexer/SpinMotorDutyCycle", dutyCycle);
    }

    public Command setSpeedCommand(double indexerSpeed, double spinMotorSpeed) {
        return Commands
            .runEnd(() -> setSpinMotorSpeed(spinMotorSpeed), () -> setSpinMotorSpeed(0), this)
            .alongWith(runEnd(() -> setIndexerSpeed(indexerSpeed), () -> setIndexerSpeed(0)));
    }
}
