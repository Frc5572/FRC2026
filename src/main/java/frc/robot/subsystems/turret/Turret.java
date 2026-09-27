package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.LinkedList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.localization.DrivetrainState;
import frc.robot.localization.TurretCameraAdapter;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.localization.TagVisibility;

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
    private static final CameraConstants TURRET_CAMERA = Arrays
        .stream(Constants.Vision.cameraConstants).filter(c -> c.isTurret).findFirst().orElseThrow();
    private double searchOffset = 0.0;
    private double searchDirection = 1.0;
    private double searchAmplitude = 0.0;
    private double lastSearchTime = Double.NaN;
    /** Capture-time cutoff: the search ends once a vision frame newer than this is fused. */
    private double searchStart = Double.NaN;
    private boolean searching = false;
    private boolean shooting = false;

    /** Whether the turret is sweeping to look for tags. */
    public boolean isSearching() {
        return searching;
    }

    /**
     * Mark whether a shot is in progress. Shooting stops any search and returns to aiming.
     *
     * @param shooting whether the robot is shooting
     */
    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }

    /** Start a search now, e.g. from an operator button. It ends once tags are found. */
    public void requestSearch() {
        searchStart = MathSharedStore.getTimestamp();
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
     * Whether the estimated pose says the turret camera should see hub tags at this aim. Past half
     * field the hub is too far away to expect tags, so the turret just aims where the estimate says
     * the hub is.
     */
    private boolean expectsTags(Rotation2d aimFieldRelative) {
        Pose2d robot = state.getGlobalPoseEstimate();
        if (AllianceFlipUtil.applyX(robot.getX()) > FieldConstants.fieldLength / 2) {
            return false;
        }
        Rotation2d turretAngle = aimFieldRelative.minus(robot.getRotation());
        Transform3d robotToCamera = new Transform3d(Constants.Vision.turretCenter.getTranslation(),
            new Rotation3d(0.0, 0.0, turretAngle.getRadians())).plus(TURRET_CAMERA.robotToCamera);
        int visible = TagVisibility.countVisible(robot, robotToCamera,
            TURRET_CAMERA.horizontalFieldOfView, Constants.Turret.searchTagMaxDistance,
            new Rotation2d(Constants.Turret.searchTagMaxIncidence), FieldConstants::isHubTag);
        return visible >= Constants.Turret.searchExpectedTags;
    }

    /**
     * Aim the turret in the field frame, sweeping around that aim to look for hub tags. A search
     * starts when the estimated pose says hub tags should be in view and the turret camera is
     * sending frames, but none has contained a hub tag for {@link Constants.Turret#searchDelay} (a
     * drifted estimate aims the camera away from the tags, so it would never be corrected
     * otherwise), or on {@link #requestSearch()}. It ends as soon as a hub tag is seen, even one
     * too poor to fuse, and never runs while shooting.
     *
     * @param rotations field-relative aim
     */
    public Command aimOrSearch(Supplier<Rotation2d> rotations) {
        return run(() -> {
            Rotation2d aim = rotations.get();
            double now = MathSharedStore.getTimestamp();
            double lastHubTag = adapter.getLastHubTagTime();
            // A dead camera is not a lost pose; searching would not help.
            boolean cameraAlive =
                now - adapter.getLastFrameTime() < Constants.Turret.searchDelay;
            boolean expectsTags = expectsTags(aim);
            Logger.recordOutput("Turret/expectsTags", expectsTags);
            Logger.recordOutput("Turret/cameraAlive", cameraAlive);
            if (Double.isNaN(searchStart) && expectsTags && cameraAlive
                && now - lastHubTag > Constants.Turret.searchDelay) {
                searchStart = now;
            }
            if (shooting || lastHubTag > searchStart) {
                searchStart = Double.NaN;
            }
            searching = !Double.isNaN(searchStart);
            Logger.recordOutput("Turret/searching", searching);
            if (!searching) {
                resetSearch();
                setGoalFieldRelative(aim);
                return;
            }
            double rate = Constants.Turret.searchRate.in(RadiansPerSecond);
            if (Double.isNaN(lastSearchTime)) {
                searchAmplitude = Constants.Turret.searchStartAmplitude.in(Radians);
            } else {
                searchOffset += searchDirection * rate * Math.min(now - lastSearchTime, 0.1);
            }
            lastSearchTime = now;
            Rotation2d robotRotation = state.getGlobalPoseEstimate().getRotation();
            double center = normalize(aim.minus(robotRotation)).getRadians();
            double target = center + searchOffset;
            // Reverse at the sweep edge, or before crossing a travel limit (which would whip).
            boolean pastEdge = Math.abs(searchOffset) >= searchAmplitude;
            // Stay inside the limits: -180 deg normalizes to +180 deg, which is itself a wrap.
            double maxTarget = Constants.Turret.maxAngle.minus(SEARCH_LIMIT_MARGIN).in(Radians);
            double minTarget = Constants.Turret.minAngle.plus(SEARCH_LIMIT_MARGIN).in(Radians);
            boolean pastLimit = target > maxTarget || target < minTarget;
            if ((pastEdge || pastLimit) && Math.signum(searchOffset) == searchDirection) {
                searchDirection = -searchDirection;
                searchAmplitude = Math.min(
                    searchAmplitude + Constants.Turret.searchAmplitudeStep.in(Radians),
                    Constants.Turret.searchMaxAmplitude.in(Radians));
            }
            // Robot rotation can also carry the target past a limit; never let it wrap.
            target = MathUtil.clamp(target, minTarget, maxTarget);
            searchOffset = target - center;
            Logger.recordOutput("Turret/searchOffsetDeg", Units.radiansToDegrees(searchOffset));
            setGoalRobotRelative(new Rotation2d(target), RadiansPerSecond
                .of(searchDirection * rate - state.getFieldRelativeSpeeds().omegaRadiansPerSecond));
        }).finallyDo(() -> {
            resetSearch();
            searching = false;
        });
    }

    private void resetSearch() {
        searchOffset = 0.0;
        searchDirection = 1.0;
        lastSearchTime = Double.NaN;
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
