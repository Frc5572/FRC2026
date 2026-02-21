package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
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

    private boolean hasSynced = false;
    private final TurretIO io;
    public final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
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

        Angle turretRotationEstimate =
            getTurretAngleFromGears(inputs.gear1AbsoluteAngle, inputs.gear2AbsoluteAngle);
        Logger.recordOutput("Turret/EstimatedTurretAngle", turretRotationEstimate);
        Logger.recordOutput("Turret/Synced", hasSynced);

        if (!hasSynced && inputs.gear1AbsoluteAngle != null) {
            io.resetPosition(turretRotationEstimate);
            hasSynced = true;
        }
        state.setTurretAngle(Timer.getTimestamp(), turretRotationEstimate);
    }

    /**
     * Estimates the turret's absolute rotation using two gear-mounted absolute encoders.
     *
     * <p>
     * This method performs a search across the valid turret angle range. For each candidate turret
     * angle, it computes the expected encoder readings and scores how closely they match the
     * measured values. The angle with the lowest error is selected.
     * </p>
     *
     * <p>
     * This approach allows reconstruction of an absolute turret angle even when individual encoders
     * wrap multiple times over the turret's range of motion.
     * </p>
     *
     * @param gear1 Absolute angle of the first gear encoder
     * @param gear2 Absolute angle of the second gear encoder
     * @return Best estimate of the turret's absolute rotation
     */
    private static Angle getTurretAngleFromGears(Rotation2d gear1, Rotation2d gear2) {
        Angle turretAngleLimited = getTurretAngleFromGearLimited(gear1,
            Constants.Turret.gear1Gearing, Constants.Turret.gear1Offset);
        Logger.recordOutput("TurretCalcs/turretAngleLimited", turretAngleLimited.in(Degrees));
        Angle step = Rotations.of(Constants.Turret.gear1Gearing);
        do {
            turretAngleLimited = turretAngleLimited.minus(step);
        } while (turretAngleLimited.gt(Constants.Turret.minAngle.getMeasure()));
        turretAngleLimited = turretAngleLimited.plus(step);
        Logger.recordOutput("TurretCalcs/Target/Gear1", gear1.getDegrees());
        Logger.recordOutput("TurretCalcs/Target/Gear2", gear2.getDegrees());
        double minScore = Double.MAX_VALUE;
        Angle currentBest = turretAngleLimited;
        int count = 0;
        while (turretAngleLimited.lt(Constants.Turret.maxAngle.getMeasure())) {
            Rotation2d gear1Guess = getGearAnglesFromTurret(turretAngleLimited,
                Constants.Turret.gear1Gearing, Constants.Turret.gear1Offset);
            Rotation2d gear2Guess = getGearAnglesFromTurret(turretAngleLimited,
                Constants.Turret.gear2Gearing, Constants.Turret.gear2Offset);
            double score = normalize(gear2.minus(gear2Guess)).getRadians();
            score = score * score;
            Logger.recordOutput("TurretCalcs/Step" + count + "/TurretAngle",
                turretAngleLimited.in(Degrees));
            Logger.recordOutput("TurretCalcs/Step" + count + "/Gear1Guess",
                gear1Guess.getDegrees());
            Logger.recordOutput("TurretCalcs/Step" + count + "/Gear2Guess",
                gear2Guess.getDegrees());
            Logger.recordOutput("TurretCalcs/Step" + count + "/score", score);
            if (score < minScore) {
                minScore = score;
                currentBest = turretAngleLimited;
            }
            turretAngleLimited = turretAngleLimited.plus(step);
            count++;
        }
        return currentBest;
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
     * Computes a turret rotation estimate from a single gear encoder, without accounting for angle
     * wrapping.
     *
     * <p>
     * This result is only valid modulo the gear ratio and must be further processed to determine
     * the true absolute turret angle.
     * </p>
     *
     * @param gear Absolute angle of the gear encoder
     * @param gearing Gear ratio between turret and encoder
     * @param offset Encoder offset applied during calibration
     * @return Turret angle estimate prior to wrap disambiguation
     */
    private static Angle getTurretAngleFromGearLimited(Rotation2d gear, double gearing,
        Rotation2d offset) {
        return gear.getMeasure().minus(offset.getMeasure()).times(gearing);
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
    public void setGoal(Rotation2d targetAngle, AngularVelocity velocity) {
        targetAngle = normalize(targetAngle);
        if (targetAngle.getRadians() > Constants.Turret.maxAngle.getRadians()) {
            targetAngle = Constants.Turret.maxAngle;
            if (velocity.in(RotationsPerSecond) > 0.0) {
                velocity = RotationsPerSecond.zero();
            }
        }
        if (targetAngle.getRadians() < Constants.Turret.minAngle.getRadians()) {
            targetAngle = Constants.Turret.minAngle;
            if (velocity.in(RotationsPerSecond) < 0.0) {
                velocity = RotationsPerSecond.zero();
            }
        }
        if (hasSynced) {
            io.setTargetAngle(targetAngle, velocity);
        }
    }

    /** Aim turret in robot frame */
    public Command goToAngleRobotRelative(Supplier<Rotation2d> rotations) {
        return run(() -> this.setGoal(rotations.get(), RotationsPerSecond.of(0)));
    }

    /** Aim turret in field frame */
    public Command goToAngleFieldRelative(Supplier<Rotation2d> rotations) {
        return run(
            () -> this.setGoal(rotations.get().minus(state.getGlobalPoseEstimate().getRotation()),
                RadiansPerSecond.of(-state.getCurrentSpeeds().omegaRadiansPerSecond)));
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
