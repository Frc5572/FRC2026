package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber subsystem for controlling the robot's climbing mechanism.
 *
 * <p>
 * This subsystem manages both the telescope extension and pivot rotation components of the climber.
 * It uses bang-bang control for precise positioning and logs all relevant data for debugging and
 * analysis.
 *
 * @see ClimberIO
 */
public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    /**
     * Constructs a new Climber subsystem.
     *
     * @param io the hardware interface for the climber subsystem
     */
    public Climber(ClimberIO io) {
        this.io = io;
    }

    /**
     * Periodic method called regularly by the command scheduler.
     *
     * <p>
     * Processes input data from hardware and logs it for monitoring.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/", inputs);
    }

    /**
     * Creates a command to move the telescope to a target height.
     *
     * <p>
     * Uses bang-bang control to extend or retract the telescope and waits until the mechanism
     * reaches within 1 inch of the target height.
     *
     * @param height a supplier providing the target height in any distance unit
     * @return a command that moves the telescope to the specified height
     */
    public Command moveToTelescope(Supplier<Distance> height) {
        return runOnce(() -> {
            Logger.recordOutput("targetHeight", height.get().in(Meters));
            io.setHeightTelescope(height.get());
        }).andThen(Commands.waitUntil(
            () -> Math.abs(inputs.positionTelescope.in(Inches) - height.get().in(Inches)) < 1));
    }

    /**
     * Creates a command to move the pivot to a target angle.
     *
     * <p>
     * Sets the pivot to the specified angle and waits until the mechanism reaches within 1 radian
     * of the target angle.
     *
     * @param angle a supplier providing the target angle in any angle unit
     * @return a command that rotates the pivot to the specified angle
     */
    public Command moveToPivot(Supplier<Angle> angle) {
        return runOnce(() -> {
            Logger.recordOutput("targetAngle", angle.get().in(Radians));
            io.setAnglePivot(angle.get());
        }).andThen(Commands.waitUntil(
            () -> Math.abs(inputs.positionPivot.in(Radians) - angle.get().in(Radians)) < 1));
    }
}
