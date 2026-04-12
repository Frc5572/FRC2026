package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        Constants.Indexer.constants.ifDirty(constants -> {
            io.setConstants(constants);
        });
    }

    public void setMagazineDutyCycle(double dutyCycle) {
        io.setMagazineDutyCycle(dutyCycle);
        Logger.recordOutput("Indexer/MagazineDutyCycle", dutyCycle);
    }

    public void setSpindexerDutyCycle(double dutyCycle) {
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
        return runEnd(() -> {
            setSpindexerDutyCycle(spindexerDutyCycle);
            setMagazineDutyCycle(magazineDutyCycle);
        }, () -> {
            setSpindexerDutyCycle(0);
            setMagazineDutyCycle(0);
        });
    }

    /** Spin indexer if doSpin is true */
    public Command runSpindexer(BooleanSupplier doSpin) {
        return runEnd(() -> {
            boolean doSpin_ = doSpin.getAsBoolean();
            double value = doSpin_ ? 1.0 : 0.0;
            setSpindexerDutyCycle(value);
            setMagazineDutyCycle(value);
        }, () -> {
            setSpindexerDutyCycle(0);
            setMagazineDutyCycle(0);
        });
    }

    /**
     * Spin indexer while intaking balls
     *
     * @return Command
     */
    public Command spinWhileIntake() {
        int[] counts = new int[] {0};
        double[] prev = new double[] {0.0};
        Command init = runOnce(() -> {
            counts[0] = 0;
        });
        Command spinIndexer = setSpeedCommand(0, 0.4).until(() -> {
            double vel = inputs.spindexerVelocity.in(RotationsPerSecond);
            boolean spinStopped = vel < 0.1;
            prev[0] = vel;
            if (spinStopped) {
                counts[0]++;
            } else {
                counts[0] = 0;
            }
            return counts[0] > 5;
        });
        return init.andThen(spinIndexer);
    }
}
