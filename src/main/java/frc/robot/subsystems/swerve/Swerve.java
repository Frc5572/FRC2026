package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.localization.DrivetrainState;
import frc.robot.subsystems.shooter.TargetingState;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.mod.SwerveModule;
import frc.robot.subsystems.swerve.mod.SwerveModuleIO;
import frc.robot.subsystems.swerve.util.MoveToPoseBuilder;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.controls.ControlsConfig;
import frc.robot.controls.ControlsField;
import frc.robot.tuning.DrivetrainTuning;
import frc.robot.tuning.LimitApplier;
import frc.robot.tuning.LimitDetectors;
import frc.robot.tuning.LimitRampSpec;
import frc.robot.tuning.StepResponse;
import frc.robot.subsystems.swerve.util.SwerveRateLimiter;
import frc.robot.subsystems.swerve.util.TeleopControls;
import frc.robot.subsystems.swerve.util.TuningCommands;
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
 * <li>Pose estimation and vision fusion via {@link TargetingState}</li>
 * <li>Acceleration, tilt, and skid limiting via {@link SwerveRateLimiter}</li>
 * </ul>
 *
 * <h2>Threading model</h2> Odometry-related sensor signals are updated on a dedicated background
 * thread. Access to these signals and derived state is synchronized using a shared
 * {@code odometryLock} to ensure consistency across the estimator and modules.
 *
 * <h2>Pose estimation</h2> Wheel encoder and gyro data are integrated at high rate to produce
 * odometry updates, which are then fused with delayed vision measurements inside
 * {@link TargetingState}. The resulting pose estimate is the authoritative source of robot position
 * for autonomous and field-relative driving.
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

    private final Lock odometryLock;
    private final PhoenixOdometryThread odometryThread;
    public final SwerveModule[] modules;
    private final GyroIO gyro;
    private final GyroInputsAutoLogged gyroInputs;
    private final SwerveIO io;
    private final SwerveInputsAutoLogged inputs;

    private final SwerveRateLimiter limiter;
    private final Supplier<ControlsConfig> controlsConfig;
    private final Supplier<DrivetrainTuning> tuningConfig;
    private DrivetrainTuning appliedTuning = DrivetrainTuning.defaults();
    private ChassisSpeeds lastCommanded = new ChassisSpeeds();
    private double lastFollowError = 0.0;
    private double lastTilt = 0.0;
    private double lastSkid = 1.0;
    private boolean tuningApplied = false;

    private boolean flipTrajectories = false;

    public final DrivetrainState state;

    public AutoFactory autoFactory;

    private static boolean sideLocked = false;

    private static boolean verticalLocked = false;

    /**
     * Simple container type that bundles together the {@link Swerve} subsystem and its associated
     * {@link DrivetrainState} estimator.
     *
     * <p>
     * This record is used to pass around both the high-level swerve drive control interface and the
     * underlying drivetrain state estimation as a single unit. It does not itself construct or
     * initialize any hardware or background processing; it only stores references to the provided
     * instances.
     *
     * @param swerve the swerve drive subsystem instance
     * @param drivetrainState the drivetrain state estimator associated with the swerve subsystem
     */
    public static record Bundle(Swerve swerve, DrivetrainState drivetrainState) {
    }

    /** Creates Swerve and DrivetrainState static factory */
    public static Bundle create(Function<PhoenixOdometryThread, SwerveIO> swerveIo,
        Function<PhoenixOdometryThread, GyroIO> gyroIo,
        BiFunction<Integer, PhoenixOdometryThread, SwerveModuleIO> moduleIoFn,
        Supplier<ControlsConfig> controlsConfig, Supplier<DrivetrainTuning> tuningConfig) {
        Lock localLock = new ReentrantLock();
        PhoenixOdometryThread localOdometryThread = new PhoenixOdometryThread(localLock);

        GyroIO localGyro = gyroIo.apply(localOdometryThread);
        GyroInputsAutoLogged localGyroInputs = new GyroInputsAutoLogged();
        SwerveIO localIo = swerveIo.apply(localOdometryThread);

        SwerveInputsAutoLogged localInputs = new SwerveInputsAutoLogged();

        SwerveModule[] localModules = IntStream.range(0, Constants.Swerve.modulesConstants.length)
            .mapToObj(i -> new SwerveModule(i, moduleIoFn.apply(i, localOdometryThread)))
            .toArray(SwerveModule[]::new);

        localOdometryThread.start();

        localLock.lock();
        SwerveModulePosition[] initPositions = new SwerveModulePosition[localModules.length];
        try {
            Arrays.stream(localModules).map(mod -> {
                mod.updateInputs();
                return mod.getPosition();
            }).toArray(_i -> initPositions);

            localGyro.updateInputs(localGyroInputs);
            Logger.processInputs("Swerve/Gyro", localGyroInputs);
        } finally {
            localLock.unlock();
        }

        DrivetrainState instantiatedState = new DrivetrainState(initPositions, localGyroInputs.yaw);

        Swerve instantiatedSwerve = new Swerve(localLock, localOdometryThread, localModules,
            localGyro, localGyroInputs, localIo, localInputs, instantiatedState, controlsConfig,
            tuningConfig);

        return new Bundle(instantiatedSwerve, instantiatedState);
    }


    private Swerve(Lock odometryLock, PhoenixOdometryThread odometryThread, SwerveModule[] modules,
        GyroIO gyro, GyroInputsAutoLogged gyroInputs, SwerveIO io, SwerveInputsAutoLogged inputs,
        DrivetrainState state, Supplier<ControlsConfig> controlsConfig,
        Supplier<DrivetrainTuning> tuningConfig) {
        super("Swerve");

        this.odometryLock = odometryLock;
        this.odometryThread = odometryThread;
        this.modules = modules;
        this.gyro = gyro;
        this.gyroInputs = gyroInputs;
        this.io = io;
        this.inputs = inputs; // B. Assign it here to fix the error!
        this.state = state;
        this.controlsConfig = controlsConfig;
        this.tuningConfig = tuningConfig;
        this.limiter = new SwerveRateLimiter(controlsConfig);

        this.autoFactory = new AutoFactory(state::getGlobalPoseEstimate, state::resetPose,
            this::followTrajectory, true, this);
    }


    /**
     * Set this to true to flip trajectories about the y axis (left/right) for auto paths.
     */
    public void flipTrajectories(boolean doFlip) {
        this.flipTrajectories = doFlip;
    }

    /**
     * Follow a Choreo Trajectory
     *
     * @param sample SwerveSample of choreo tajectory
     */
    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = state.getGlobalPoseEstimate();
        PIDController xController = Constants.Swerve.holonomicDriveController.getXController();
        PIDController yController = Constants.Swerve.holonomicDriveController.getYController();
        ProfiledPIDController thetaController =
            Constants.Swerve.holonomicDriveController.getThetaController();
        if (flipTrajectories) {
            sample = new SwerveSample(sample.t, sample.x, FieldConstants.fieldWidth - sample.y,
                -sample.heading, sample.vx, -sample.vy, -sample.omega, sample.ax, -sample.ay,
                -sample.alpha, sample.moduleForcesX(), sample.moduleForcesY());
        }
        // Generate the next speeds for the robot
        ChassisSpeeds speeds =
            new ChassisSpeeds(sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y), sample.omega
                    + thetaController.calculate(pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        driveFieldRelative(speeds);
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

        applyTuning();

        double[] sampleTimestamps = this.inputs.timestamps;
        SwerveModulePosition[] wheelPositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < sampleTimestamps.length; i++) {
            for (int j = 0; j < modules.length; j++) {
                wheelPositions[j] = modules[j].getOdometryPosition(i);
            }
            state.addOdometryObservation(wheelPositions,
                Rotation2d.fromRadians(gyroInputs.yawRads[i]), sampleTimestamps[i]);
        }
        SwerveModuleState[] wheelStates = new SwerveModuleState[modules.length];
        for (int j = 0; j < modules.length; j++) {
            wheelStates[j] = modules[j].getState();
        }
        ChassisSpeeds currentSpeeds =
            Constants.Swerve.swerveKinematics.toChassisSpeeds(wheelStates);
        limiter.update(currentSpeeds);
        state.updateMeasuredSpeeds(currentSpeeds);
        publishDetectors(lastCommanded, currentSpeeds);

        Logger.recordOutput("Swerve/GlobalPoseEstimate", state.getGlobalPoseEstimate());

        // targetingState.updateTargeting();
    }

    /**
     * A square-wave velocity step test on the drive motors.
     *
     * <p>
     * Holds every module pointed straight ahead and alternates the commanded wheel speed between
     * zero and the configured amplitude, logging setpoint against measured and reporting rise
     * time, overshoot and steady-state error for each half-cycle. This is the measurement half of
     * the feedforward-then-feedback procedure: with the feedback gains at zero, a good physical
     * model shows up as a small steady-state error.
     *
     * <p>
     * Deliberately bypasses {@link SwerveRateLimiter} — the chassis acceleration limits would
     * shape the very transient being measured.
     *
     * @param tuning supplies the step amplitude and period
     * @return the step-test command, which runs until cancelled
     */
    public Command driveVelocityStepTest(Supplier<DrivetrainTuning> tuning) {
        Timer timer = new Timer();
        boolean[] high = {false};
        double[] stepStart = {0.0};
        StepResponse[] current = {null};
        return this.run(() -> {
            DrivetrainTuning cfg = tuning.get();
            double period = cfg.stepPeriod();
            double amplitude = cfg.stepAmplitude();
            double measured = measuredForwardSpeed();

            if (timer.get() >= period) {
                if (current[0] != null) {
                    current[0].log("Tuning/DriveStep");
                }
                high[0] = !high[0];
                stepStart[0] = measured;
                current[0] =
                    new StepResponse(measured, high[0] ? amplitude : 0.0, period);
                timer.restart();
            }

            double setpoint = high[0] ? amplitude : 0.0;
            if (current[0] != null) {
                current[0].accept(timer.get(), measured);
            }
            Logger.recordOutput("Tuning/DriveStep/Setpoint", setpoint);
            Logger.recordOutput("Tuning/DriveStep/Measured", measured);

            SwerveModuleState[] states = new SwerveModuleState[modules.length];
            for (int i = 0; i < modules.length; i++) {
                states[i] = new SwerveModuleState(setpoint, Rotation2d.kZero);
            }
            for (int i = 0; i < modules.length; i++) {
                modules[i].setDesiredState(states[i]);
            }
        }).beforeStarting(() -> {
            timer.restart();
            high[0] = false;
            stepStart[0] = 0.0;
            current[0] = null;
        }).finallyDo(() -> {
            if (current[0] != null) {
                current[0].log("Tuning/DriveStep");
            }
            setModuleStates(new ChassisSpeeds());
        });
    }

    /** Mean forward wheel speed across the modules, in meters per second. */
    private double measuredForwardSpeed() {
        double sum = 0.0;
        for (SwerveModule module : modules) {
            sum += module.getState().speedMetersPerSecond;
        }
        return sum / modules.length;
    }

    /** Commanded-versus-measured shortfall, the forward-limit failure condition, in m/s. */
    public double detectorFollowError() {
        return lastFollowError;
    }

    /** Chassis tilt from level, the tilt-limit failure condition, in degrees. */
    public double detectorTilt() {
        return lastTilt;
    }

    /** Module max-to-median translational ratio, the skid-limit failure condition. */
    public double detectorSkidRatio() {
        return lastSkid;
    }

    /** Publish the three limit-procedure detectors, so they can be watched while driving. */
    private void publishDetectors(ChassisSpeeds commanded, ChassisSpeeds measured) {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        double follow = LimitDetectors.followError(
            Math.hypot(commanded.vxMetersPerSecond, commanded.vyMetersPerSecond),
            Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond));
        double tilt = LimitDetectors.tiltDegrees(gyroInputs.pitch, gyroInputs.roll);
        double skid = LimitDetectors.skidRatio(states, Constants.Swerve.swerveTranslations,
            measured.omegaRadiansPerSecond);
        Logger.recordOutput("Tuning/Detect/FollowError", follow);
        Logger.recordOutput("Tuning/Detect/TiltDegrees", tilt);
        Logger.recordOutput("Tuning/Detect/SkidRatio", skid);
        lastFollowError = follow;
        lastTilt = tilt;
        lastSkid = skid;
    }

    /**
     * Step an acceleration limit up or down until its failure condition appears.
     *
     * <p>
     * This automates 1690's three limit procedures (software-sessions.md, lines 157&ndash;176),
     * which are each "change the limit until X happens". The robot drives a straight burst,
     * watches the relevant detector, then steps the limit and repeats. The limits not under test
     * are opened right up for the duration so the one being measured is the binding constraint,
     * and everything is restored when the command ends.
     *
     * <p>
     * <strong>This drives the robot.</strong> Each burst covers roughly two to three metres, so
     * it needs about six metres of clear floor.
     *
     * @param spec which limit to ramp and what counts as failure
     * @param limits applies candidate limits to the active driver profile
     * @return the ramp command, which runs until it trips or is cancelled
     */
    public Command accelerationLimitRamp(LimitRampSpec spec, LimitApplier limits) {
        Timer phase = new Timer();
        double[] candidate = {spec.start()};
        double[] peak = {0.0};
        double[] lastGood = {Double.NaN};
        boolean[] driving = {true};
        double[] saved = new double[ControlsField.values().length];
        boolean[] savedEnabled = new boolean[ControlsField.values().length];

        return this.run(() -> {
            if (driving[0]) {
                setModuleStates(limiter.limit(new ChassisSpeeds(
                    controlsConfig.get().translationMaxSpeed(), 0.0, 0.0)));
                peak[0] = Math.max(peak[0], spec.detector().getAsDouble());
                if (phase.get() >= spec.burstSeconds()) {
                    driving[0] = false;
                    phase.restart();
                }
                return;
            }

            setModuleStates(new ChassisSpeeds());
            if (phase.get() < spec.settleSeconds()) {
                return;
            }

            boolean tripped = spec.trippedWhenAbove()
                ? peak[0] > spec.threshold() : peak[0] <= spec.threshold();
            Logger.recordOutput("Tuning/Ramp/Candidate", candidate[0]);
            Logger.recordOutput("Tuning/Ramp/Peak", peak[0]);
            Logger.recordOutput("Tuning/Ramp/Tripped", tripped);

            if (tripped) {
                // Ramping up, the answer is the last value that survived; ramping down, it is the
                // first value that came back clean.
                double answer = spec.step() > 0 ? lastGood[0] : candidate[0];
                Logger.recordOutput("Tuning/Ramp/Recommended", answer);
                Logger.recordOutput("Tuning/Ramp/Complete", true);
                driving[0] = false;
                return;
            }

            lastGood[0] = candidate[0];
            candidate[0] += spec.step();
            if (candidate[0] > spec.field().maximum() || candidate[0] < spec.field().minimum()) {
                Logger.recordOutput("Tuning/Ramp/Recommended", lastGood[0]);
                Logger.recordOutput("Tuning/Ramp/Complete", true);
                return;
            }
            limits.setValue(spec.field(), candidate[0]);
            peak[0] = 0.0;
            driving[0] = true;
            phase.restart();
        }).beforeStarting(() -> {
            ControlsConfig cfg = controlsConfig.get();
            for (ControlsField f : ControlsField.values()) {
                saved[f.ordinal()] = cfg.get(f);
                savedEnabled[f.ordinal()] = cfg.isEnabled(f);
            }
            // Open the other limits so only the one under test binds.
            for (ControlsField f : spec.othersToOpen()) {
                limits.setValue(f, f.maximum());
                limits.setEnabled(f, false);
            }
            candidate[0] = spec.start();
            peak[0] = 0.0;
            lastGood[0] = Double.NaN;
            driving[0] = true;
            // The skid and tilt limits ship switched off; a disabled limit reports the "no limit"
            // sentinel whatever its value, so the one under test has to be switched on.
            limits.setValue(spec.field(), candidate[0]);
            limits.setEnabled(spec.field(), true);
            Logger.recordOutput("Tuning/Ramp/Complete", false);
            phase.restart();
        }).finallyDo(() -> {
            for (ControlsField f : ControlsField.values()) {
                limits.setValue(f, saved[f.ordinal()]);
                limits.setEnabled(f, savedEnabled[f.ordinal()]);
            }
            setModuleStates(new ChassisSpeeds());
        });
    }

    /**
     * Push tuning changes down to the modules.
     *
     * <p>
     * Each call is a blocking CAN configuration write across eight motor controllers, so it only
     * fires when the configuration actually changes. Gains come from the tuning config rather
     * than {@code Constants}, so there is a single authority and it is the one the log records.
     */
    private void applyTuning() {
        DrivetrainTuning tuning = tuningConfig.get();
        if (tuningApplied && tuning.equals(appliedTuning)) {
            return;
        }
        appliedTuning = tuning;
        tuningApplied = true;
        for (SwerveModule module : modules) {
            module.setDriveGains(tuning);
            module.setAngleGains(tuning);
        }
        Logger.recordOutput("Swerve/TuningAppliedAt", Timer.getFPGATimestamp());
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
            speeds = limiter.limit(speeds);
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
        return driveRobotRelative(() -> {
            ChassisSpeeds speeds = driveSpeeds.get();
            if (sideLocked) {
                Rotation2d currentRotation = this.state.getGlobalPoseEstimate().getRotation();
                // normalize between (-180, 180]
                double rotationTarget = currentRotation.getDegrees() < 0 ? -90 : 90;
                Rotation2d rotationError =
                    Rotation2d.fromDegrees(rotationTarget).minus(currentRotation);
                double omega = rotationError.getRadians() * 5.0;
                omega = Math.max(-Constants.Swerve.maxAngularVelocity,
                    Math.min(Constants.Swerve.maxAngularVelocity, omega));
                speeds.omegaRadiansPerSecond = omega;
            } else if (verticalLocked) {
                Rotation2d currentRotation = this.state.getGlobalPoseEstimate().getRotation();
                // normalize between (-180, 180]
                double rotationTarget = Math.abs(currentRotation.getDegrees()) < 90 ? 0 : 180;
                Rotation2d rotationError =
                    Rotation2d.fromDegrees(rotationTarget).minus(currentRotation);
                double omega = rotationError.getRadians() * 5.0;
                omega = Math.max(-Constants.Swerve.maxAngularVelocity,
                    Math.min(Constants.Swerve.maxAngularVelocity, omega));
                speeds.omegaRadiansPerSecond = omega;
            }
            return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getUserRelativeHeading());
        });
    }

    /**
     * Drives the robot using the estimated global field heading.
     *
     * <p>
     * The supplied field-relative speeds are transformed into robot-relative speeds using the
     * current pose estimate from {@link DrivetrainState}.
     *
     * @param driveSpeeds supplier of field-relative chassis speeds
     * @return a command that drives the robot while scheduled
     */
    public Command driveFieldRelative(Supplier<ChassisSpeeds> driveSpeeds) {
        return driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeeds.get(),
            state.getGlobalPoseEstimate().getRotation()));
    }

    private void driveFieldRelative(ChassisSpeeds driveSpeeds) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeeds,
            state.getGlobalPoseEstimate().getRotation());
        // speeds = limiter.limit(speeds);
        setModuleStates(speeds);
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
     * {@link DrivetrainState#resetPose(Pose2d)} instead.
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
        var builder = new MoveToPoseBuilder(this, (speeds) -> {
            setModuleStates(limiter.limit(speeds));
        });
        return builder;
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
     * Characterize the drive feedforward and hand the fitted gains to a callback, so a tuning
     * run persists instead of ending in a copy-paste into {@code Constants}.
     *
     * @param onResult receives (kS, kV) in volts and volts per rad/s
     * @return the characterization command
     */
    public Command feedforwardCharacterization(TuningCommands.DoubleBinaryConsumer onResult) {
        return TuningCommands.feedforwardCharacterization(this, this::runCharacterization,
            this::getFFCharacterizationVelocity, onResult);
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
            .plus(AllianceFlipUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero));
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
            var speeds = limiter.limit(new ChassisSpeeds());
            return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1;
        }).andThen(this.emergencyStop());
    }


    /**
     * Get Position on field from Odometry
     *
     * @return Pose2d on the field
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return state.getGlobalPoseEstimate();
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

    private void setModuleStates(ChassisSpeeds chassisSpeeds) {
        this.lastCommanded = chassisSpeeds;
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

    /** X the wheels. */
    public Command wheelsIn() {
        SwerveModuleState[] states =
            new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-135))};
        return run(() -> {
            for (int i = 0; i < modules.length; i++) {
                modules[i].setDesiredState(states[i]);
            }
        });
    }


    /**
     * Creates a command that drives and aims at the side walls
     *
     * @return drive-and-shoot command
     */
    public Command driveFacingSides(DoubleSupplier forward, DoubleSupplier right, double maxSpeed,
        double maxRotSpeed) {
        return this.driveUserRelative(() -> {
            double omega = 0.0;
            double xaxis = right.getAsDouble();
            double yaxis = forward.getAsDouble();
            var cfg = controlsConfig.get();
            double sx = Math.signum(xaxis);
            double sy = Math.signum(yaxis);
            xaxis = TeleopControls.shapeMagnitude(Math.abs(xaxis), cfg.translationDeadband(),
                cfg.translationSaturation(), cfg.translationCurve(), cfg.translationExponent())
                * sx;
            yaxis = TeleopControls.shapeMagnitude(Math.abs(yaxis), cfg.translationDeadband(),
                cfg.translationSaturation(), cfg.translationCurve(), cfg.translationExponent())
                * sy;
            Rotation2d currentRotation = this.state.getGlobalPoseEstimate().getRotation();
            // normalize between (-180, 180]
            double normalizedAngle = ((currentRotation.getDegrees() + 180) % 360 + 360) % 360 - 180;
            double rotationError = normalizedAngle < 0 ? -90 : 90;
            omega = rotationError * 5.0;
            omega = Math.max(-Constants.Swerve.maxAngularVelocity,
                Math.min(Constants.Swerve.maxAngularVelocity, omega));
            ChassisSpeeds fieldRelative = new ChassisSpeeds(xaxis, yaxis, omega);
            return fieldRelative;
        });
    }

    /**
     * Toggle Side lock
     *
     * @return Command
     */
    public Command toggleSideLock() {
        return Commands.startEnd(() -> sideLocked = true, () -> sideLocked = false);
    }

    /**
     * Toggle Vertical lock
     */
    public Command toggleVerticalLock() {
        return Commands.startEnd(() -> verticalLocked = true, () -> verticalLocked = false);
    }

}
