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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathSharedStore;
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
    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
    public final TurretCameraAdapter adapter =
        new TurretCameraAdapter(Constants.Vision.turretCenter.getTranslation());
    private final DrivetrainState state;

    /** Last commanded robot-relative setpoint in rotations, or NaN if not position-controlled. */
    private double lastSetpoint = Double.NaN;
    private boolean whipping = false;
    private double whipStart = 0.0;
    private double prevAngle = Double.NaN;
    private double prevAngleTime = 0.0;
    private final LinearFilter rateFilter = LinearFilter.movingAverage(5);

    /**
     * Creates a new Turret subsystem.
     *
     * @param io Hardware abstraction used to read sensors and control actuators
     */
    public Turret(TurretIO io, DrivetrainState state) {
        super("Turret");
        this.io = io;
        this.state = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Constants.Turret.pid.ifDirty(io::setPID);

        Logger.recordOutput("Turret/currentAngle", inputs.relativeAngle);
        double now = MathSharedStore.getTimestamp();
        double rate = 0.0;
        if (!Double.isNaN(prevAngle) && now > prevAngleTime) {
            rate = rateFilter
                .calculate(Math.abs(inputs.relativeAngle - prevAngle) / (now - prevAngleTime));
        }
        prevAngle = inputs.relativeAngle;
        prevAngleTime = now;
        // The whip ends when the turret reaches its setpoint, stops moving (it may never reach an
        // unreachable or stale setpoint), or runs out of time.
        if (whipping && (Double.isNaN(lastSetpoint)
            || Math.abs(lastSetpoint - inputs.relativeAngle) < WHIP_SETTLED.in(Rotations)
            || (now - whipStart > WHIP_GRACE && rate < WHIP_STOPPED.in(RotationsPerSecond))
            || now - whipStart > WHIP_TIMEOUT)) {
            whipping = false;
        }
        Logger.recordOutput("Turret/isWhipping", whipping);
        Logger.recordOutput("Turret/searching", searching);
        adapter.recordTurretAngle(MathSharedStore.getTimestamp(),
            new Rotation2d(Rotations.of(inputs.relativeAngle)), whipping);
        // Marks the robot as moving for DrivetrainState's stationary check.
        state.setTurretRawAngle(MathSharedStore.getTimestamp(), Rotations.of(inputs.relativeAngle));
    }

    private static final Angle WHIP_SETTLED = Degrees.of(10);
    private static final AngularVelocity WHIP_STOPPED = DegreesPerSecond.of(45);
    /** Seconds after a setpoint wrap before a stopped turret ends the whip. */
    private static final double WHIP_GRACE = 0.2;
    private static final double WHIP_TIMEOUT = 1.5;

    /**
     * Whether the turret is swinging the long way around because its setpoint wrapped past a travel
     * limit. Shooting and turret-camera vision should be paused while this is true.
     */
    public boolean isWhipping() {
        return whipping;
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

    public void setVoltageIO(DoubleSupplier voltage) {
        lastSetpoint = Double.NaN;
        io.setTurretVoltage(Volts.of(voltage.getAsDouble()));
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
        if (!Double.isNaN(lastSetpoint) && Math.abs(setpoint - lastSetpoint) > 0.5) {
            whipping = true;
            whipStart = MathSharedStore.getTimestamp();
        }
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
        return !whipping && !searching && !Double.isNaN(lastSetpoint)
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
            start[0] = MathSharedStore.getTimestamp();
            lastTime[0] = start[0];
            // Start from where the turret is, heading toward the limit with more travel left.
            target[0] = MathUtil.clamp(getTurretHeading().getRadians(), minTarget, maxTarget);
            direction[0] = maxTarget - target[0] >= target[0] - minTarget ? 1.0 : -1.0;
        }).andThen(run(() -> {
            double now = MathSharedStore.getTimestamp();
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
