package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public final class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void setShootingSpeed(double speed) {
        io.setShootingSpeed(speed);
    }

    public void periodic() {
        io.updateInputs(inputs);
        // You can log inputs here if needed
    }

    public Command runShooter() {
        return Commands.runEnd(() -> setShootingSpeed(0.0), () -> setShootingSpeed(0.0), this);
    }
}
