package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Tuples.Tuple2;

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
    private final ClimberIO io;
    public final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

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
        Logger.processInputs("Climber", inputs);
        Constants.Climber.Pivot.pidConstants.ifDirty(io::setPIDPivot);
        Constants.Climber.Telescope.pidConstants.ifDirty(io::setPIDTelescope);
    }

    /**
     * Set target climber state.
     */
    public Command moveTo(Supplier<Tuple2<Angle, Distance>> state) {
        return this.run(() -> {
            var next = state.get();
            this.io.setAnglePivot(next._0());
            this.io.setHeightTelescope(next._1());
        });
    }

    /**
     * Raise climber to the correct angle
     *
     * @return Command
     */
    public Command raiseClimber() {
        return runOnce(() -> this.io.setAnglePivot(Constants.Climber.Pivot.DEGREES_AT_TOP))
            .andThen(Commands.waitUntil(() -> this.inputs.positionPivot
                .isNear(Constants.Climber.Pivot.DEGREES_AT_TOP, Degrees.of(5))));
    }

    /**
     * Climb to Level 1
     *
     * @return Command
     */
    public Command climbLevel1() {
        return runOnce(() -> this.io.setHeightTelescope(Constants.Climber.Telescope.LEVEL_1_CLIMB))
            .andThen(Commands.waitUntil(() -> this.inputs.positionTelescope
                .isNear(Constants.Climber.Telescope.LEVEL_1_CLIMB, Inches.of(2))));
    }
}
