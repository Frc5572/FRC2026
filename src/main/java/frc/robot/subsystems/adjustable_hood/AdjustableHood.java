package frc.robot.subsystems.adjustable_hood;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.Supplier;
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

        Logger.recordOutput("AdjustableHood/ActualAngle (Deg)", inputs.relativeAngle.in(Degrees));
    }

    public Command moveWithVoltage(double voltage) {
        return run(() -> io.setAdjustableHoodVoltage(voltage));
    }

    public Command setGoal(Angle setAngle) {
        return runOnce(() -> io.setTargetAngle(setAngle));
    }

    public Command setGoal(Supplier<Angle> setAngle) {
        return runOnce(() -> io.setTargetAngle(setAngle.get()));
    }
}
