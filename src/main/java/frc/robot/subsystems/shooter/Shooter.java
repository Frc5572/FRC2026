package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 * Shooter Subsystem
 */
public final class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private Debouncer torqueCurrentDebouncer = new Debouncer(0.1, DebounceType.kFalling);

    /**
     * Shooter Subsystem Constructor
     *
     * @param io Shooter IO implementation
     */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Constants.Shooter.constants.ifDirty(constants -> {
            io.setConstants(constants);
            torqueCurrentDebouncer =
                new Debouncer(constants.atSpeedDebounce, DebounceType.kFalling);
        });
    }

    public void setVelocity(double velocity) {
        boolean inTolerance = Math.abs(inputs.shooterAngularVelocity1.in(RotationsPerSecond)
            - velocity) <= Constants.Shooter.constants.velocityTolerance;
        boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
        if (torqueCurrentControl) {
            io.runTorqueCurrentVelocity(velocity);
        } else {
            io.runDutyCycleVelocity(velocity);
        }
    }

    public Command shoot(double velocity) {
        return run(() -> setVelocity(velocity));
    }
}
