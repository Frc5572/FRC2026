package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Shooter Subsystem
 */
public final class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

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
    }

    public Command runShooterVelocityCommand(double velocity) {
        return run(() -> io.runShooterVelocity(velocity));
    }
}
