package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Shooter Subsystem
 */
public final class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIO.ShooterInputs inputs;

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
    }

    public Command runShooterVelocityCommand(double velocityRPM) {
        return run(() -> io.runShooterVelocity(velocityRPM));
    }
}
