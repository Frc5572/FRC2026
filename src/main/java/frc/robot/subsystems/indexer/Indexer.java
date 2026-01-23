package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    IndexerIO io;
    IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    private void setIndexerSpeed(double speed) {
        io.setIndexerSpeed(speed);
        Logger.recordOutput("Indexer/IndexerSpeed", speed);
    }

    private void setSpinMotorSpeed(double speed) {
        io.setSpinMotorSpeed(speed);
        Logger.recordOutput("Indexer/SpinMotorSpeed", speed);
    }

    public Command setSpeedCommand(double indexerSpeed, double spinMotorSpeed) {
        return Commands
            .runEnd(() -> setSpinMotorSpeed(spinMotorSpeed), () -> setSpinMotorSpeed(0), this)
            .alongWith(runEnd(() -> setIndexerSpeed(indexerSpeed), () -> setIndexerSpeed(0)));
    }
}
