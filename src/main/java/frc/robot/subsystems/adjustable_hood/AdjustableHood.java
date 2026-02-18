package frc.robot.subsystems.adjustable_hood;

import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Adjustable Hood Subsystem
 */
public class AdjustableHood extends SubsystemBase {

    private final AdjustableHoodIO io;
    public final AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();

    /**
     * Creates a new Adjustable Hood subsystem.
     *
     * @param io Hardware abstraction
     */
    public AdjustableHood(AdjustableHoodIO io) {
        super("Adjustable Hood");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Adjustable Hood", inputs);
    }

    /**
     * @param targetAngle gets the goal angle
     */
    public void setGoal(Angle targetAngle) {
        io.setTargetAngle(targetAngle);
    }

    public Command goToAngle(Angle angle) {
        return run(() -> this.setGoal(angle));
    }

    public Command runVoltage(DoubleSupplier voltage) {
        return this.run(() -> {
            io.setAdjustableHoodVoltage(Volts.of(voltage.getAsDouble()));
        });
    }
}
