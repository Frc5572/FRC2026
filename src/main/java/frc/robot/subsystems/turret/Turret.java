package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

/**
 * Subsystem representing the robot turret.
 */
public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
    private final RobotState state;

    /**
     * Creates a new Turret subsystem.
     *
     * @param io Hardware abstraction used to read sensors and control actuators
     */
    public Turret(TurretIO io, RobotState state) {
        super("Turret");
        this.io = io;
        this.state = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Constants.Turret.pid.ifDirty(io::setPID);

        Logger.recordOutput("Turret/CancoderAngle",
            inputs.gear2AbsoluteAngle.div(Constants.Turret.gear2Gearing).in(Degrees));

        state.setTurretRawAngle(Timer.getTimestamp(), Rotations.of(inputs.relativeAngle));
    }

    public Rotation2d getTurretHeading() {
        return Rotation2d.fromRotations(this.inputs.relativeAngle);
    }

    /**
     * Normalizes a rotation to the range (-pi, pi].
     *
     * @param rot Rotation to normalize
     * @return Normalized rotation
     */
    private static Rotation2d normalize(Rotation2d rot) {
        return new Rotation2d(rot.getCos(), rot.getSin());
    }

    /** Set turret motor's output voltage. */
    public Command setVoltage(DoubleSupplier voltage) {
        return this.run(() -> {
            io.setTurretVoltage(Volts.of(voltage.getAsDouble()));
        });
    }

    /**
     *
     * @param targetAngle gets the goal angle
     */
    public boolean setGoalRobotRelative(Rotation2d targetAngle, AngularVelocity velocity) {
        var normalized = normalize(targetAngle).getMeasure();
        if (normalized.lt(Constants.Turret.minAngle)) {
            normalized = normalized.plus(Rotations.of(1));
        }
        if (normalized.gt(Constants.Turret.maxAngle)) {
            normalized = normalized.minus(Rotations.of(1));
        }
        io.setTargetAngle(normalized, velocity);
        return true;
    }

    /** Set target angle relative to the field. */
    public boolean setGoalFieldRelative(Rotation2d targetAngle) {
        return this.setGoalRobotRelative(
            targetAngle.minus(state.getGlobalPoseEstimate().getRotation()),
            RadiansPerSecond.of(-state.getFieldRelativeSpeeds().omegaRadiansPerSecond));
    }

    /** Aim turret in robot frame */
    public Command goToAngleRobotRelative(Supplier<Rotation2d> rotations) {
        return run(() -> this.setGoalRobotRelative(rotations.get(), RotationsPerSecond.of(0)));
    }

    /** Aim turret in field frame */
    public Command goToAngleFieldRelative(Supplier<Rotation2d> rotations) {
        return run(() -> this.setGoalFieldRelative(rotations.get()));
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
                io.setTurretVoltage(Volts.of(0.0));
                Logger.recordOutput("Sysid/Turret/FF/appliedVoltage", 0.0);
            }).withTimeout(1.5),
            // Start timer
            this.runOnce(timer::restart),
            // Accelerate and gather data
            this.run(() -> {
                double voltage = timer.get() * 0.1;
                Logger.recordOutput("Sysid/Turret/FF/appliedVoltage", voltage);
                io.setTurretVoltage(Volts.of(voltage));
                velocitySamples.add(inputs.velocity.in(RotationsPerSecond));
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
