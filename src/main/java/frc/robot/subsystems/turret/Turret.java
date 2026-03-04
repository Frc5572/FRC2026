package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
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
    // inputs.rotation + offset = actual rotation
    private double offset = 0.0;

    /**
     * Creates a new Turret subsystem.
     *
     * @param io Hardware abstraction used to read sensors and control actuators
     */
    public Turret(TurretIO io, RobotState state) {
        super("Turret");
        this.io = io;
        this.state = state;
        this.state.setTurretOffsetUpdate(newOffset -> {
            Logger.recordOutput("Turret/offset", newOffset);
            this.offset = newOffset;
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Constants.Turret.pid.ifDirty(io::setPID);

        state.setTurretRawAngle(Timer.getTimestamp(), inputs.relativeAngle);
    }

    public Rotation2d getTurretHeading() {
        return new Rotation2d(this.inputs.relativeAngle).plus(Rotation2d.fromRotations(offset));
    }

    /**
     * Computes the expected gear encoder angle for a given turret rotation.
     *
     * @param rotation Absolute turret rotation
     * @param gearing Gear ratio between turret and encoder
     * @param offset Encoder offset applied during calibration
     * @return Normalized expected encoder angle
     */
    public static Rotation2d getGearAnglesFromTurret(Angle rotation, double gearing,
        Rotation2d offset) {
        return normalize(new Rotation2d(rotation.div(gearing).plus(offset.getMeasure())));
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

    /**
     *
     * @param targetAngle gets the goal angle
     */
    public boolean setGoalRobotRelative(Rotation2d targetAngle, AngularVelocity velocity) {
        targetAngle = normalize(targetAngle);
        if (isValidAngle(targetAngle) || isValidAngle(targetAngle.plus(Rotation2d.fromRotations(1)))
            || isValidAngle(targetAngle.plus(Rotation2d.fromRotations(-1)))) {
            io.setTargetAngle(targetAngle.minus(Rotation2d.fromRotations(offset)), velocity);
            Logger.recordOutput("Turret/InRange", true);
            Logger.recordOutput("Turret/ActualTarget", targetAngle);
            return true;
        }
        Logger.recordOutput("Turret/InRange", false);
        double minAngleDiff =
            fmod((targetAngle.getDegrees() - Constants.Turret.minAngle.getDegrees()) + 180, 360)
                - 180;
        double maxAngleDiff =
            fmod((targetAngle.getDegrees() - Constants.Turret.maxAngle.getDegrees()) + 180, 360)
                - 180;
        if (Math.abs(minAngleDiff) < Math.abs(maxAngleDiff)) {
            io.setTargetAngle(Constants.Turret.minAngle.minus(Rotation2d.fromRotations(offset)),
                RotationsPerSecond.of(0));
            Logger.recordOutput("Turret/ActualTarget", Constants.Turret.minAngle);
        } else {
            io.setTargetAngle(Constants.Turret.maxAngle.minus(Rotation2d.fromRotations(offset)),
                RotationsPerSecond.of(0));
            Logger.recordOutput("Turret/ActualTarget", Constants.Turret.maxAngle);
        }
        return false;
    }

    /** Set target angle relative to the field. */
    public boolean setGoalFieldRelative(Rotation2d targetAngle) {
        return this.setGoalRobotRelative(
            targetAngle.minus(state.getGlobalPoseEstimate().getRotation()).plus(Rotation2d.k180deg),
            RadiansPerSecond.of(-state.getFieldRelativeSpeeds().omegaRadiansPerSecond));
    }

    private boolean isValidAngle(Rotation2d targetAngle) {
        if (targetAngle.getRadians() > Constants.Turret.maxAngle.getRadians()) {
            return false;
        }
        if (targetAngle.getRadians() < Constants.Turret.minAngle.getRadians()) {
            return false;
        }
        return true;
    }

    private static double fmod(double a, double n) {
        return a - Math.floor(a / n) * n;
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
