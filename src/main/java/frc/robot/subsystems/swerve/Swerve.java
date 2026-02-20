package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Radians;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.mod.SwerveModule;
import frc.robot.subsystems.swerve.mod.SwerveModuleIO;
import frc.robot.subsystems.swerve.util.MoveToPoseBuilder;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.util.SwerveRateLimiter;
import frc.robot.subsystems.swerve.util.TuningCommands;
import frc.robot.subsystems.swerve.util.TurnToRotation;
import frc.robot.util.AllianceFlipUtil;

/**
 * Primary swerve drivetrain subsystem.
 *
 * <p>
 * This subsystem owns and coordinates all components required to control and estimate the state of
 * a swerve drive, including:
 * <ul>
 * <li>Swerve modules and their IO implementations</li>
 * <li>Gyro integration</li>
 * <li>High-frequency odometry sampling via {@link PhoenixOdometryThread}</li>
 * <li>Pose estimation and vision fusion via {@link RobotState}</li>
 * <li>Acceleration, tilt, and skid limiting via {@link SwerveRateLimiter}</li>
 * </ul>
 *
 * <h2>Threading model</h2> Odometry-related sensor signals are updated on a dedicated background
 * thread. Access to these signals and derived state is synchronized using a shared
 * {@code odometryLock} to ensure consistency across the estimator and modules.
 *
 * <h2>Pose estimation</h2> Wheel encoder and gyro data are integrated at high rate to produce
 * odometry updates, which are then fused with delayed vision measurements inside
 * {@link RobotState}. The resulting pose estimate is the authoritative source of robot position for
 * autonomous and field-relative driving.
 *
 * <h2>Driving model</h2> All drive commands ultimately resolve to robot-relative
 * {@link ChassisSpeeds}. These speeds are passed through a {@link SwerveRateLimiter} before being
 * discretized and converted to per-module states.
 *
 * <p>
 * This class exposes convenience commands for robot-relative, field-relative, and user-relative
 * driving, as well as trajectory-style pose targeting and characterization routines.
 */
@NullMarked
public final class Swerve extends SubsystemBase {

    private final Lock odometryLock = new ReentrantLock();
    private final PhoenixOdometryThread odometryThread;
    public final SwerveModule[] modules;
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private final SwerveIO io;
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    private final SwerveRateLimiter limiter = new SwerveRateLimiter();

    public final RobotState state;

    public double customSkidLimit = 1000.0;

    /**
     * Constructs the swerve subsystem and initializes all hardware interfaces, estimator state, and
     * background odometry processing.
     *
     * <p>
     * The provided factories are invoked with a shared {@link PhoenixOdometryThread} instance to
     * allow sensors and motors to register high-frequency signals for synchronized sampling.
     *
     * @param swerveIo factory for creating the drivetrain-level IO implementation
     * @param gyroIo factory for creating the gyro IO implementation
     * @param moduleIoFn factory for creating per-module IO implementations
     */
    public Swerve(Function<PhoenixOdometryThread, SwerveIO> swerveIo,
        Function<PhoenixOdometryThread, GyroIO> gyroIo,
        BiFunction<Integer, PhoenixOdometryThread, SwerveModuleIO> moduleIoFn) {
        super("Swerve");
        this.odometryThread = new PhoenixOdometryThread(this.odometryLock);
        this.gyro = gyroIo.apply(this.odometryThread);
        this.modules = IntStream.range(0, Constants.Swerve.modulesConstants.length)
            .mapToObj(i -> new SwerveModule(i, moduleIoFn.apply(i, this.odometryThread)))
            .toArray(SwerveModule[]::new);
        this.io = swerveIo.apply(odometryThread);
        this.odometryThread.start();
        this.odometryLock.lock();
        SwerveModulePosition[] initPositions = new SwerveModulePosition[modules.length];
        try {
            Arrays.stream(modules).map(mod -> {
                mod.updateInputs();
                return mod.getPosition();
            }).toArray(_i -> initPositions);
        } finally {
            this.odometryLock.unlock();
        }
        this.state = new RobotState(initPositions, this.gyroInputs.yaw);
    }

    @Override
    public void periodic() {
        this.odometryLock.lock();

        for (int i = 0; i < modules.length; i++) {
            this.modules[i].updateInputs();
        }

        this.gyro.updateInputs(this.gyroInputs);
        Logger.processInputs("Swerve/Gyro", this.gyroInputs);

        this.io.updateInputs(this.inputs);
        Logger.processInputs("Swerve/Timestamps", this.inputs);

        this.odometryLock.unlock();

        for (int i = 0; i < modules.length; i++) {
            this.modules[i].periodic();
        }

        SwerveModuleState[] wheelStates = new SwerveModuleState[modules.length];
        for (int j = 0; j < modules.length; j++) {
            wheelStates[j] = modules[j].getState();
        }
        ChassisSpeeds currentSpeeds =
            Constants.Swerve.swerveKinematics.toChassisSpeeds(wheelStates);
        double[] sampleTimestamps = this.inputs.timestamps;
        double gyroPerSample = currentSpeeds.omegaRadiansPerSecond / sampleTimestamps.length;
        SwerveModulePosition[] wheelPositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < sampleTimestamps.length; i++) {
            for (int j = 0; j < modules.length; j++) {
                wheelPositions[j] = modules[j].getOdometryPosition(i);
            }
            state.addOdometryObservation(wheelPositions,
                gyroInputs.connected ? Rotation2d.fromRadians(gyroInputs.yawRads[i])
                    : state.getGlobalPoseEstimate().getRotation()
                        .plus(Rotation2d.fromRadians((gyroPerSample + 1) * i)),
                sampleTimestamps[i]);
        }
        limiter.update(currentSpeeds);
        state.updateSpeeds(currentSpeeds);

        Logger.recordOutput("Swerve/GlobalPoseEstimate", state.getGlobalPoseEstimate());
    }

    /**
     * Drives the robot using robot-relative chassis speeds.
     *
     * <p>
     * Supplied speeds are passed through the {@link SwerveRateLimiter} before being applied to the
     * drivetrain.
     *
     * @param driveSpeeds supplier of robot-relative chassis speeds
     * @return a command that drives the robot while scheduled
     */
    public Command driveRobotRelative(Supplier<ChassisSpeeds> driveSpeeds) {
        return this.run(() -> {
            ChassisSpeeds speeds = driveSpeeds.get();
            speeds = limiter.limit(speeds, customSkidLimit);
            setModuleStates(speeds);
        });
    }

    /**
     * Drives the robot using a user-defined field reference heading.
     *
     * <p>
     * The supplied field-relative speeds are transformed into robot-relative speeds using the
     * user-controlled heading offset.
     *
     * @param driveSpeeds supplier of field-relative chassis speeds
     * @return a command that drives the robot while scheduled
     */
    public Command driveUserRelative(Supplier<ChassisSpeeds> driveSpeeds) {
        return driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeeds.get(),
            getUserRelativeHeading()));
    }

    /**
     * Drives the robot using the estimated global field heading.
     *
     * <p>
     * The supplied field-relative speeds are transformed into robot-relative speeds using the
     * current pose estimate from {@link RobotState}.
     *
     * @param driveSpeeds supplier of field-relative chassis speeds
     * @return a command that drives the robot while scheduled
     */
    public Command driveFieldRelative(Supplier<ChassisSpeeds> driveSpeeds) {
        return driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeeds.get(),
            state.getGlobalPoseEstimate().getRotation()));
    }

    /**
     * Immediately sets the robot's pose to a new value, updating both the odometry estimator and
     * the underlying simulation state (if any).
     *
     * <p>
     * This is useful when you need to forcibly override the robot's pose, for example during
     * testing or at the start of autonomous in simulation.
     *
     * <p>
     * If you only want to update the odometry/estimator without affecting any simulation state, use
     * {@link RobotState#resetPose(Pose2d)} instead.
     *
     * @param newPose a supplier that provides the new robot pose in field coordinates
     * @return a command that applies the pose override once when scheduled
     */
    public Command overridePose(Supplier<Pose2d> newPose) {
        return Commands.runOnce(() -> {
            Pose2d newPose_ = newPose.get();
            io.resetPose(newPose_);
            state.resetPose(newPose_);
        });
    }

    /**
     * Creates a command builder for driving the robot to a target global pose.
     *
     * <p>
     * The generated command uses a holonomic controller and the current pose estimate to compute
     * chassis speeds, which are rate-limited and applied to the drivetrain.
     *
     * @return a {@link MoveToPoseBuilder} for configuring pose targets
     */
    public MoveToPoseBuilder moveToPose() {
        return new MoveToPoseBuilder(this, (speeds) -> {
            speeds = limiter.limit(speeds, customSkidLimit);
            setModuleStates(speeds);
        });
    }

    /**
     * Creates a SysId routine for drivetrain feedforward characterization.
     *
     * <p>
     * This routine is used to identify kS and kV parameters for the swerve drive by applying
     * controlled voltage steps and measuring resulting motion.
     *
     * @return a command that runs feedforward characterization
     */
    public Command feedforwardCharacterization() {
        return TuningCommands.feedforwardCharacterization(this, this::runCharacterization,
            this::getFFCharacterizationVelocity);
    }

    /**
     * Creates a SysId routine for wheel radius characterization.
     *
     * <p>
     * This routine estimates the effective wheel radius by correlating commanded motion with
     * measured yaw change.
     *
     * @return a command that runs wheel radius characterization
     */
    public Command wheelRadiusCharacterization() {
        return TuningCommands.wheelRadiusCharacterization(this, this::setModuleStates,
            this::getWheelRadiusCharacterizationPositions, () -> this.gyroInputs.yaw);
    }

    /**
     * Sets a user-defined field-relative heading offset.
     *
     * <p>
     * This offset is used by user-relative driving modes to define "forward" independently of the
     * robot's pose estimate.
     *
     * @param knownHeading supplier of the desired field heading
     * @return a one-shot command that applies the offset
     */
    public Command setFieldRelativeOffset(Supplier<Rotation2d> knownHeading) {
        return Commands.runOnce(
            () -> fieldOffset = gyroInputs.yaw.getRotations() - knownHeading.get().getRotations());
    }

    /**
     * Sets a user-defined field-relative heading offset.
     *
     * <p>
     * This offset is used by user-relative driving modes to define "forward" independently of the
     * robot's pose estimate.
     *
     * @return a one-shot command that sets the offset such that the current direction is "forward"
     */
    public Command setFieldRelativeOffset() {
        return setFieldRelativeOffset(() -> Rotation2d.kZero);
    }

    /**
     * Sets a user-defined field-relative heading offset.
     *
     * <p>
     * This offset is used by user-relative driving modes to define "forward" independently of the
     * robot's pose estimate.
     *
     * @return a one-shot command that sets the offset such that it agrees with the estimated pose
     *         (+180 degrees when on red alliance)
     */
    public Command resetFieldRelativeOffsetBasedOnPose() {
        return setFieldRelativeOffset(() -> state.getGlobalPoseEstimate().getRotation()
            .plus(AllianceFlipUtil.shouldFlip() ? Rotation2d.kZero : Rotation2d.k180deg));
    }

    /**
     * Creates a command that smoothly brings the drivetrain to a complete stop.
     *
     * <p>
     * This command commands zero desired chassis speeds and allows the {@link SwerveRateLimiter} to
     * decelerate the robot within configured acceleration, tilt, and skid constraints rather than
     * stopping abruptly.
     *
     * <p>
     * The command completes once the rate-limited translational speed falls below a small
     * threshold, indicating the robot is effectively stationary. After completion, a final
     * zero-speed command is issued to ensure all modules are explicitly commanded to stop.
     *
     * <p>
     * This is intended for use when transitioning between driving modes, autonomous steps, or
     * before actions that require the robot to be fully settled.
     *
     * @return a command that decelerates and stops the drivetrain
     */
    public Command stop() {
        return this.driveRobotRelative(ChassisSpeeds::new).until(() -> {
            var speeds = limiter.limit(new ChassisSpeeds(), customSkidLimit);
            return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1;
        }).andThen(this.emergencyStop());
    }

    /**
     * Creates a command that immediately commands zero chassis speeds to the drivetrain.
     *
     * <p>
     * This method bypasses all rate limiting and deceleration constraints and directly commands the
     * swerve modules to stop in a single control cycle. As a result, it may cause abrupt
     * deceleration, increased state estimation error, and/or loss of traction depending on robot
     * speed and surface conditions.
     *
     * <p>
     * <b>In most situations, {@link #stop()} should be preferred</b>, as it brings the robot to
     * rest in a controlled manner using the {@link SwerveRateLimiter}.
     *
     * <p>
     * This command is intended only for exceptional circumstances such as fault handling, disable
     * transitions, or safety-critical interruptions where immediate cessation of motion is
     * required.
     *
     * @return a command that immediately commands zero chassis speeds
     */
    public Command emergencyStop() {
        return this.runOnce(() -> setModuleStates(new ChassisSpeeds()));
    }

    public Command limitSkidLimit() {
        return Commands.runEnd(() -> customSkidLimit = FieldConstants.Hub.innerWidth / 2.0,
            () -> customSkidLimit = 1000.0, this);
    }

    private void runCharacterization(double output) {
        for (SwerveModule module : modules) {
            module.runCharacterization(output);
        }
    }

    private double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (SwerveModule module : modules) {
            output += module.getFFCharacterizationVelocity() / modules.length;
        }
        return output;
    }

    private double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    private double fieldOffset = 0.0;

    /**
     * Returns the current user-relative heading used for driving.
     *
     * <p>
     * This heading is derived from the gyro yaw and a manually controlled field offset, independent
     * of the pose estimator.
     *
     * @return user-relative field heading
     */
    public Rotation2d getUserRelativeHeading() {
        return Rotation2d.fromRotations(gyroInputs.yaw.getRotations() - fieldOffset);
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(swerveModuleStates);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    /** Identifies Closest Trench */
    public Translation2d closestTrench() {
        Pose2d botPosition = state.getGlobalPoseEstimate();
        Translation2d botLocation = botPosition.getTranslation();

        return botLocation.nearest(Constants.Trench.trenchLocation);
    }

    /** Returns the distance between the robot and the closest Trench zone. */
    public double distanceFromClosestTrench() {
        Translation2d target = closestTrench();
        double distance = state.getGlobalPoseEstimate().getTranslation().getDistance(target);

        Logger.recordOutput("Swerve/DistanceToTrench", distance);

        return distance;
    }

    /** Returns true if the robot is in the Trench zone. */
    public boolean inTrench() {
        Translation2d botLocation = state.getGlobalPoseEstimate().getTranslation();
        double botXLocation = botLocation.getX();
        double botYLocation = botLocation.getY();
        if (closestTrench().equals(FieldConstants.LeftTrench.redTrenchCenterLeft)) {
            return botXLocation >= (FieldConstants.LeftTrench.redCloseCenterLeft).getX()
                && botXLocation <= (FieldConstants.LeftTrench.redFarCenterLeft).getX()
                && botYLocation >= (FieldConstants.LeftTrench.openingTopRight).getY()
                && botYLocation <= (FieldConstants.LeftTrench.openingTopLeft).getY();
        } else if (closestTrench().equals(FieldConstants.LeftTrench.blueTrenchCenterLeft)) {
            return botXLocation >= (FieldConstants.LeftTrench.blueCloseCenterLeft).getX()
                && botXLocation <= (FieldConstants.LeftTrench.blueFarCenterLeft).getX()
                && botYLocation >= (FieldConstants.LeftTrench.openingTopRight).getY()
                && botYLocation <= (FieldConstants.LeftTrench.openingTopLeft).getY();
        } else if (closestTrench().equals(FieldConstants.RightTrench.redTrenchCenterRight)) {
            return botXLocation >= (FieldConstants.RightTrench.redCloseCenterRight).getX()
                && botXLocation <= (FieldConstants.RightTrench.redFarCenterRight).getX()
                && botYLocation >= (FieldConstants.RightTrench.openingTopRight).getY()
                && botYLocation <= (FieldConstants.RightTrench.openingTopLeft).getY();
        } else if (closestTrench().equals(FieldConstants.RightTrench.blueTrenchCenterRight)) {
            return botXLocation >= (FieldConstants.RightTrench.blueCloseCenterRight).getX()
                && botXLocation <= (FieldConstants.RightTrench.blueFarCenterRight).getX()
                && botYLocation >= (FieldConstants.RightTrench.openingTopRight).getY()
                && botYLocation <= (FieldConstants.RightTrench.openingTopLeft).getY();
        } else {
            return false;
        }
    }

    /** returns a list with all of the trench bounds */
    public static final List<Translation2d> trenchSides() {
        List<Translation2d> trenchLocations = List.of(FieldConstants.LeftTrench.redCloseCenterLeft,
            FieldConstants.LeftTrench.redFarCenterLeft,
            FieldConstants.LeftTrench.blueCloseCenterLeft,
            FieldConstants.LeftTrench.blueFarCenterLeft,
            FieldConstants.RightTrench.redCloseCenterRight,
            FieldConstants.RightTrench.redFarCenterRight,
            FieldConstants.RightTrench.blueFarCenterRight,
            FieldConstants.RightTrench.blueCloseCenterRight);
        return trenchLocations;
    }

    /** Returns the goal location of the robot */
    public Translation2d goalTrenchPosition() {
        Translation2d currentBotPos = state.getGlobalPoseEstimate().getTranslation();

        return currentBotPos.nearest(trenchSides());
    }

    /** Command that moves the robot to the trench without going through */
    public Command moveToTrench() {
        return this.moveToPose()
            .target(
                () -> new Pose2d(goalTrenchPosition(), state.getGlobalPoseEstimate().getRotation()))
            .autoRoutine(null).maxSpeed(Constants.Swerve.maxSpeed).flipForRed(false)
            .translationTolerance(Constants.Trench.tolerance).rotationTolerance(Math.toRadians(2))
            .finish().until(() -> this.inTrench());
    }

    private final PIDController pidYController =
        new PIDController(Constants.SwerveTransformPID.translationP,
            Constants.SwerveTransformPID.translationI, Constants.SwerveTransformPID.translationD);

    /**
     * Command that moves the robot through the trench, constraining Y movement to stay in the
     * trench.
     * 
     * @param driveSpeeds supplier of field-relative chassis speeds
     * @return a command that drives the robot while scheduled
     */
    public Command moveThroughTrench(Supplier<ChassisSpeeds> driveSpeeds) {
        double pidYVal = pidYController.calculate(
            state.getGlobalPoseEstimate().getTranslation().getY(), goalTrenchPosition().getY());
        return driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(driveSpeeds.get().vxMetersPerSecond, pidYVal,
                driveSpeeds.get().omegaRadiansPerSecond),
            state.getGlobalPoseEstimate().getRotation()));
    }

    /**
     * Creates a command that rotates the robot to face the hub and then locks the swerve modules in
     * a cross configuration.
     *
     *
     * The robot first turns to the angle opposite the hub direction. After turning, the modules are
     * set to fixed angles to resist motion.
     *
     * @return command that aims and locks the drivetrain
     */
    public Command pointAtHubAndCross() {
        Supplier<Rotation2d> anglePose =
            () -> new Rotation2d(Radians.of(FieldConstants.Hub.innerCenterPoint.toTranslation2d()
                .minus(state.getGlobalPoseEstimate().getTranslation()).getAngle().getRadians()))
                    .rotateBy(Rotation2d.k180deg);

        return new TurnToRotation(this, anglePose, false).andThen(Commands.run(() -> {
            modules[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)));
            modules[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)));
            modules[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)));
            modules[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)));
        }));
    }

}
