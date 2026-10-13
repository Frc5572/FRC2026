package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.localization.DrivetrainState;
import frc.robot.localization.TurretCameraAdapter;

/**
 * Subsystem representing the robot turret.
 */
public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final Lock odometryLock;
    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
    public final TurretCameraAdapter adapter =
        new TurretCameraAdapter(Constants.Vision.turretCenter.getTranslation());
    private final Timer timer = new Timer();
    private final DrivetrainState state;
    private boolean currentlyWhipping = false;
    private double rotationGoal;

    /** Last commanded robot-relative setpoint in rotations, or NaN if not position-controlled. */
    private double lastSetpoint = Double.NaN;

    /**
     * Creates a new Turret subsystem.
     *
     * @param io Hardware abstraction used to read sensors and control actuators
     */
    public Turret(TurretIO io, DrivetrainState state, Lock odometryLock) {
        super("Turret");
        this.io = io;
        this.state = state;
        this.odometryLock = odometryLock;
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        io.updateInputs(inputs);
        odometryLock.unlock();
        Logger.processInputs("Turret", inputs);

        Constants.Turret.pid.ifDirty(io::setPID);

        Logger.recordOutput("Turret/isWhipping", isWhipping());
        Logger.recordOutput("Turret/currentAngle", inputs.relativeAngle);
        Logger.recordOutput("Turret/targetAngle", rotationGoal);
        Logger.recordOutput("Turret/searching", searching);

        if (currentlyWhipping) {
            if (Units.rotationsToDegrees(Math
                .abs(rotationGoal - inputs.relativeAngle)) <= Constants.Turret.degreeTolerance) {
                currentlyWhipping = false;
                Logger.recordOutput("Turret/whipFailed", false);
            }

            if (timer.hasElapsed(Constants.Turret.maxWhipTime)) {
                currentlyWhipping = false;
                Logger.recordOutput("Turret/whipFailed", !(Units.rotationsToDegrees(Math.abs(
                    rotationGoal - inputs.relativeAngle)) <= Constants.Turret.degreeTolerance));
            }
        }

        for (int i = 0; i < inputs.timestamps.length; i++) {
            adapter.recordTurretAngle(inputs.timestamps[i],
                new Rotation2d(Rotations.of(inputs.angleRotations[i])), isWhipping());
            state.setTurretRawAngle(inputs.timestamps[i], Rotations.of(inputs.angleRotations[i]));
        }
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
            setVoltageIO(voltage);
        });
    }

    /** Set turret motor's output voltage, automatically whip ends. */
    public void setVoltageIO(DoubleSupplier voltage) {
        lastSetpoint = Double.NaN;
        io.setTurretVoltage(Volts.of(voltage.getAsDouble()));
        if (currentlyWhipping) {
            Logger.recordOutput("Turret/whipFailed", true);
        }
        currentlyWhipping = false;
    }

    /**
     *
     * @param targetAngle gets the goal angle
     */
    public boolean setGoalRobotRelative(Rotation2d targetAngle, AngularVelocity velocity) {
        Logger.recordOutput("Turret/targetAngle", targetAngle);
        var normalized = normalize(targetAngle).getMeasure();
        if (normalized.lt(Constants.Turret.minAngle)) {
            normalized = normalized.plus(Rotations.of(1));
        }
        if (normalized.gt(Constants.Turret.maxAngle)) {
            normalized = normalized.minus(Rotations.of(1));
        }
        double setpoint = normalized.in(Rotations);
        // Prevent wrap chatter at +/- 180 degrees: if the target is hovering near the rear limit,
        // stay on the side of the limit matching the turret's current position to avoid whipping back and forth.
        if (Math.abs(setpoint) > 0.48) {
            double current = inputs.relativeAngle;
            if (current > 0 && setpoint < 0) {
                setpoint = 0.5;
                normalized = Rotations.of(0.5);
            } else if (current < 0 && setpoint > 0) {
                setpoint = -0.5;
                normalized = Rotations.of(-0.5);
            }
        }
        if (Math.abs(rotationGoal - normalized.in(Rotations)) >= Units.degreesToRotations(180)) {
            currentlyWhipping = true;
            timer.restart();
        }
        rotationGoal = normalized.in(Rotations);
        lastSetpoint = setpoint;
        io.setTargetAngle(normalized, velocity);
        return true;
    }

    /** Set target angle relative to the field. */
    public boolean setGoalFieldRelative(Rotation2d targetAngle) {
        return this.setGoalRobotRelative(
            targetAngle.minus(state.getGlobalPoseEstimate().getRotation()),
            RadiansPerSecond.of(-state.getFieldRelativeSpeeds().omegaRadiansPerSecond));
    }

    private static final Angle SEARCH_LIMIT_MARGIN = Degrees.of(5);
    private boolean searching = false;
    private boolean shooting = false;

    /** Whether the turret is sweeping to look for tags. */
    public boolean isSearching() {
        return searching;
    }

    /**
     * Mark whether a shot is in progress. Shooting ends any search.
     *
     * @param shooting whether the robot is shooting
     */
    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }

    /**
     * Whether the turret is where it was told to go: not whipping around, not searching, and within
     * {@link Constants.Turret#aimTolerance} of its setpoint.
     */
    public boolean isAimed() {
        return !isWhipping() && !searching && !Double.isNaN(lastSetpoint)
            && Math.abs(lastSetpoint - inputs.relativeAngle) < Constants.Turret.aimTolerance
                .in(Rotations);
    }

    /**
     * Sweep the turret's full travel in the robot frame to look for hub tags, for when the pose
     * estimate has drifted far enough that normal aiming never points the camera at them. The sweep
     * ignores the pose estimate, since anything anchored to a wrong heading would miss the tags.
     * Ends as soon as a hub tag is seen (even one too poor to fuse) or a shot starts.
     */
    public Command search() {
        double[] target = new double[1];
        double[] direction = new double[1];
        double[] lastTime = new double[1];
        double[] start = new double[1];
        double rate = Constants.Turret.searchRate.in(RadiansPerSecond);
        // Stay inside the limits: -180 deg normalizes to +180 deg, which is itself a wrap.
        double maxTarget = Constants.Turret.maxAngle.minus(SEARCH_LIMIT_MARGIN).in(Radians);
        double minTarget = Constants.Turret.minAngle.plus(SEARCH_LIMIT_MARGIN).in(Radians);
        return runOnce(() -> {
            searching = true;
            start[0] = Timer.getTimestamp();
            lastTime[0] = start[0];
            // Start from where the turret is, heading toward the limit with more travel left.
            target[0] = MathUtil.clamp(getTurretHeading().getRadians(), minTarget, maxTarget);
            direction[0] = maxTarget - target[0] >= target[0] - minTarget ? 1.0 : -1.0;
        }).andThen(run(() -> {
            double now = Timer.getTimestamp();
            target[0] += direction[0] * rate * Math.min(now - lastTime[0], 0.1);
            lastTime[0] = now;
            if (target[0] >= maxTarget) {
                target[0] = maxTarget;
                direction[0] = -1.0;
            } else if (target[0] <= minTarget) {
                target[0] = minTarget;
                direction[0] = 1.0;
            }
            Logger.recordOutput("Turret/searchTargetDeg", Units.radiansToDegrees(target[0]));
            setGoalRobotRelative(new Rotation2d(target[0]),
                RadiansPerSecond.of(direction[0] * rate));
        })).until(() -> shooting || adapter.getLastHubTagTime() > start[0])
            .finallyDo(() -> searching = false);
    }

    /** Tracks if turret is currently whipping */
    public boolean isWhipping() {
        return currentlyWhipping;
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
                lastSetpoint = Double.NaN;
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

    public void resetTurret() {
        io.resetPosition(Degrees.of(0));
    }
}
