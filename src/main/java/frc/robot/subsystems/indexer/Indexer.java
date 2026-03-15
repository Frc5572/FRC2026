package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    /**
     * Run characterization procedure
     */
    public Command characterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            this.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),
            // Let indexer stop
            this.run(() -> {
                io.setVoltageSpindexer(Volts.of(0.0));
                Logger.recordOutput("Sysid/Turret/FF/appliedVoltage", 0.0);
            }).withTimeout(1.5),
            // Start timer
            this.runOnce(timer::restart),
            // Accelerate and gather data
            this.run(() -> {
                double voltage = timer.get() * 0.1;
                Logger.recordOutput("Sysid/Turret/FF/appliedVoltage", voltage);
                io.setVoltageSpindexer(Volts.of(voltage));
                velocitySamples.add(inputs.spindexerVelocity.in(RotationsPerSecond));
                voltageSamples.add(voltage);
            }).finallyDo(() -> {
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                Logger.recordOutput("Sysid/Turret/FF/kS", kS);
                Logger.recordOutput("Sysid/Turret/FF/kV", kV);
            }));
    }

}
