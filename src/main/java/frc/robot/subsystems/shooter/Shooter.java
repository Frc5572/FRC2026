package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 * Shooter Subsystem
 */
public final class Shooter extends SubsystemBase {
    private final ShooterIO io;
    public final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private Debouncer torqueCurrentDebouncer = new Debouncer(0.1, DebounceType.kFalling);

    private LinearFilter flywheelSpeedFilter = LinearFilter.movingAverage(10);
    private double lastShot = 0.0;
    private boolean shooting = false;

    /**
     * Shooter Subsystem Constructor
     *
     * @param io Shooter IO implementation
     */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        SmartDashboard.putBoolean("Shooter/UpToSpeed", inputs.shooterAngularVelocity1
            .in(RadiansPerSecond) > Constants.Shooter.atSpeedThreshold);
        Constants.Shooter.constants.ifDirty(constants -> {
            io.setConstants(constants);
            torqueCurrentDebouncer =
                new Debouncer(constants.atSpeedDebounce, DebounceType.kFalling);
        });
        if (shooting && inputs.shooterAngularVelocity1.in(RotationsPerSecond) < flywheelSpeedFilter
            .calculate(inputs.shooterAngularVelocity1.in(RotationsPerSecond)) - 3.0) {
            lastShot = Timer.getFPGATimestamp();
        }
    }

    /** Get time since shooter last had a ball pass through. */
    public double timeSinceLastShot() {
        return Timer.getFPGATimestamp() - lastShot;
    }

    /** Set shooter velocity */
    public void setVelocity(double velocity) {
        if (velocity > 0.0) {
            shooting = true;
        } else {
            shooting = false;
        }
        boolean inTolerance = Math.abs(inputs.shooterAngularVelocity1.in(RotationsPerSecond)
            - velocity) <= Constants.Shooter.constants.velocityTolerance;
        boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
        if (torqueCurrentControl) {
            io.runTorqueCurrentVelocity(velocity);
        } else {
            io.runDutyCycleVelocity(velocity);
        }
    }

    /** Shoot at a given velocity */
    public Command shoot(double velocity) {
        return run(() -> setVelocity(velocity));
    }

    /** Shoot at a given velocity */
    public Command shoot(DoubleSupplier velocity) {
        return run(() -> setVelocity(velocity.getAsDouble()));
    }
}
