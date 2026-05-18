package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;


/**
 * Shooter Subsystem
 */
public final class Shooter extends SubsystemBase {
    private final ShooterIO io;
    public final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private final RobotState state;
    private Debouncer torqueCurrentDebouncer = new Debouncer(0.1, DebounceType.kFalling);

    private LinearFilter flywheelSpeedFilter = LinearFilter.movingAverage(10);
    private double lastShot = 0.0;
    private boolean shooting = false;

    /**
     * Shooter Subsystem Constructor
     *
     * @param io Shooter IO implementation
     */
    public Shooter(ShooterIO io, RobotState state) {
        this.io = io;
        this.state = state;
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
        state.setFlywheelSpeed(inputs.shooterAngularVelocity1.in(RotationsPerSecond));
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
        if (velocity > 0.001) {
            shooting = true;
            io.runVelocity(velocity);
        } else {
            shooting = false;
            io.runVolts(0.0);
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

    /**
     * Run characterization procedure
     *
     * <p>
     * WARNING: will not respect min/max turret angles. Unplug everything from the turret so it can
     * spin a potentially infinite number of times.
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
            // Let turret stop
            this.run(() -> {
                io.runVolts(0.0);
                Logger.recordOutput("Sysid/Turret/FF/appliedVoltage", 0.0);
            }).withTimeout(1.5),
            // Start timer
            this.runOnce(timer::restart),
            // Accelerate and gather data
            this.run(() -> {
                double voltage = timer.get() * 0.1;
                Logger.recordOutput("Sysid/Turret/FF/appliedVoltage", voltage);
                io.runVolts(voltage);
                velocitySamples.add(inputs.shooterAngularVelocity1.in(RotationsPerSecond));
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
