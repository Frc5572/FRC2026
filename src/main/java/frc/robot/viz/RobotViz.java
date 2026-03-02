package frc.robot.viz;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Arrays;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.ShotData;
import frc.robot.ShotData.ShotEntry;
import frc.robot.sim.SimulatedRobotState;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.CameraConstants;

/**
 * Centralized visualization helper for publishing robot state to logging and visualization tools.
 *
 * <p>
 * This class is responsible for publishing robot poses, mechanism transforms, and subsystem
 * geometry to {@link Logger} for visualization tools such as AdvantageScope. It computes both
 * estimated and (when available) ground-truth representations of the robot state.
 * </p>
 *
 * <h2>Estimated vs Ground-Truth Data</h2>
 * <ul>
 * <li><b>Estimated</b> data is sourced from live subsystems (swerve, turret, hood, etc.) and
 * reflects what the robot believes its state to be.</li>
 * <li><b>Ground-truth</b> data is sourced from {@link SimulatedRobotState} when running in
 * simulation. When not simulating, ground-truth outputs are aliased to the estimated state.</li>
 * </ul>
 *
 * <p>
 * All outputs are published via {@link Logger} and are intended strictly for debugging, analysis,
 * and visualization (e.g., AdvantageScope). No control or decision-making logic should depend on
 * this class.
 */
@NullMarked
public class RobotViz {

    private static final int hopperIndex = 0;
    private static final int intakeIndex = 1;
    private static final int climberIndex = 2;
    private static final int hooksIndex = 3;
    private static final int turretIndex = 4;
    private static final int hoodIndex = 5;
    private static final int flIndex = 6;
    private static final int numPoses = 10;

    private static final Translation3d turretCenter = new Translation3d(-0.1651, 0, 0.36772);
    private static final Translation3d hoodRotationCenter =
        new Translation3d(-0.078322, 0, 0.494257);
    private static final Translation3d intakeRotationCenter =
        new Translation3d(0.149203, 0, 0.245623);
    private static final Translation3d climberRotationCenter =
        new Translation3d(-0.258763, 0, 0.198437);
    private static final Distance hooksDown = Meters.of(0.5);

    private final Supplier<Pose3d> robotPoseSupplier;
    private final Supplier<Pose3d> estPoseSupplier;
    private final Supplier<Rotation2d> turretSupplier;
    private final Supplier<Rotation2d> estTurretSupplier;
    private final Runnable gtUpdate;
    private final Runnable estUpdate;
    private final Pose3d[] gtState;
    private final Pose3d[] estState = new Pose3d[numPoses];

    /**
     * Creates a new visualization helper.
     *
     * <p>
     * The visualization system automatically switches between estimated-only and
     * estimated-plus-ground-truth modes depending on whether a simulation state is provided.
     * </p>
     *
     * @param sim simulation state used for ground-truth visualization; may be {@code null}
     * @param swerve live swerve subsystem providing pose estimates
     * @param turret live turret subsystem
     * @param hood adjustable hood subsystem
     * @param intake intake subsystem
     * @param climber climber subsystem
     */
    public RobotViz(@Nullable SimulatedRobotState sim, Swerve swerve, Turret turret,
        AdjustableHood hood, Intake intake, Climber climber, Shooter shooter) {
        estPoseSupplier = () -> new Pose3d(swerve.state.getGlobalPoseEstimate());
        estUpdate = () -> {
            updateState("Viz/Est", swerve.state.getGlobalPoseEstimate(),
                swerve.state.getFieldRelativeSpeeds(),
                shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond), estState,
                turret.getTurretHeading(), hood.inputs.relativeAngle, climber.inputs.positionPivot,
                climber.inputs.positionTelescope, intake.inputs.leftHopperPosition,
                Arrays.stream(swerve.modules).map(mod -> mod.inputs.anglePosition)
                    .toArray(Rotation2d[]::new));
        };
        estTurretSupplier = () -> turret.getTurretHeading();
        if (sim == null) {
            gtState = estState;
            gtUpdate = () -> {
            };
            robotPoseSupplier = estPoseSupplier;
            turretSupplier = estTurretSupplier;
        } else {
            gtState = new Pose3d[numPoses];
            gtUpdate = () -> {
                updateState("Viz/Actual", sim.swerveDrive.mapleSim.getSimulatedDriveTrainPose(),
                    sim.swerveDrive.mapleSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    sim.shooter.flywheel.position, gtState,
                    Rotation2d.fromRadians(sim.turret.turrentAngle.position),
                    hood.inputs.relativeAngle, climber.inputs.positionPivot,
                    climber.inputs.positionTelescope, intake.inputs.leftHopperPosition,
                    Arrays.stream(swerve.modules).map(mod -> mod.inputs.anglePosition)
                        .toArray(Rotation2d[]::new));
            };
            robotPoseSupplier = () -> sim.getGroundTruthPose();
            turretSupplier = () -> Rotation2d.fromRadians(sim.turret.turrentAngle.position);
        }
    }

    private static void updateState(String trajectoryPrefix, Pose2d swervePose,
        ChassisSpeeds chassisSpeeds, double flywheelSpeed, Pose3d[] out, Rotation2d turretAngle,
        Angle hoodAngle, Angle climberAngle, Distance climberHeight, Distance intakeOut,
        Rotation2d[] modules) {

        out[hoodIndex] = new Pose3d()
            .rotateAround(hoodRotationCenter, new Rotation3d(0, hoodAngle.in(Radians), 0))
            .rotateAround(turretCenter, new Rotation3d(0, 0, turretAngle.getRadians()));
        out[turretIndex] =
            new Pose3d().rotateAround(turretCenter, new Rotation3d(0, 0, turretAngle.getRadians()));

        out[hooksIndex] =
            new Pose3d(0, 0, climberHeight.in(Meters) - hooksDown.in(Meters), Rotation3d.kZero)
                .rotateAround(climberRotationCenter,
                    new Rotation3d(0, climberAngle.in(Radians), 0));
        out[climberIndex] = new Pose3d().rotateAround(climberRotationCenter,
            new Rotation3d(0, climberAngle.in(Radians), 0));

        double start = 0.26;
        double end = 0.37;
        double t = intakeOut.in(Meters);
        double tp = (t - start) / (end - start);
        double rot = Units.degreesToRadians(25.0) * tp;
        if (tp > 1.0) {
            rot = Units.degreesToRadians(25.0);
        } else if (tp < 0.0) {
            rot = 0.0;
        }

        out[intakeIndex] =
            new Pose3d().rotateAround(intakeRotationCenter, new Rotation3d(0, rot, 0));
        out[intakeIndex] = new Pose3d(t, 0, 0, out[intakeIndex].getRotation());

        out[hopperIndex] = new Pose3d(t, 0, 0, Rotation3d.kZero);

        for (int i = 0; i < 4; i++) {
            out[flIndex + i] =
                new Pose3d().rotateAround(new Translation3d(Constants.Swerve.swerveTranslations[i]),
                    new Rotation3d(modules[i]));
        }

        drawTrajectory(trajectoryPrefix + "Trajectory", flywheelSpeed, hoodAngle.in(Degrees),
            turretAngle, swervePose, chassisSpeeds);
    }

    /**
     * Publishes visualization data for the current control loop iteration.
     *
     * <p>
     * This method should be called periodically (e.g., from {@code robotPeriodic}).
     */
    public void periodic() {
        Pose3d robotPose = robotPoseSupplier.get();
        Pose3d estPose = estPoseSupplier.get();
        Rotation2d turret = turretSupplier.get();
        Rotation2d estTurret = estTurretSupplier.get();
        Transform3d turretTransform = new Transform3d(-1.651, 0, 0, new Rotation3d(turret));
        Transform3d estTurretTransform = new Transform3d(-1.651, 0, 0, new Rotation3d(estTurret));
        Logger.recordOutput("Viz/ActualPose", robotPose);
        for (CameraConstants constants : Constants.Vision.cameraConstants) {
            if (constants.isTurret) {
                Logger.recordOutput("Viz/Cameras/" + constants.name + "/ActualPose",
                    robotPose.plus(turretTransform).plus(constants.robotToCamera));
                Logger.recordOutput("Viz/Cameras/" + constants.name + "/EstPose",
                    estPose.plus(estTurretTransform).plus(constants.robotToCamera));
            } else {
                Logger.recordOutput("Viz/Cameras/" + constants.name + "/ActualPose",
                    robotPose.plus(constants.robotToCamera));
                Logger.recordOutput("Viz/Cameras/" + constants.name + "/EstPose",
                    estPose.plus(constants.robotToCamera));
            }
        }
        Logger.recordOutput("Viz/GlobalEstPose", estPose);

        this.estUpdate.run();
        this.gtUpdate.run();
        Logger.recordOutput("Viz/EstState", estState);
        Logger.recordOutput("Viz/ActualState", gtState);
    }

    private static void drawTrajectory(String name, double flywheelSpeed, double hoodAngle,
        Rotation2d turretAngle, Pose2d swervePose, ChassisSpeeds fieldSpeeds) {
        Rotation2d effectiveTurretAngle = swervePose.getRotation().plus(turretAngle);

        double distance = ShotData.flywheelHoodToDistance.query(flywheelSpeed, hoodAngle);
        double tof = ShotData.flywheelHoodToTof.query(flywheelSpeed, hoodAngle);
        ShotData.ShotEntry entry = new ShotEntry(distance, flywheelSpeed, hoodAngle, tof);
        double vert = entry.verticalVelocity();
        double horiz = entry.horizontalVelocity();
        double horiz_x = effectiveTurretAngle.getCos() * horiz + fieldSpeeds.vxMetersPerSecond;
        double horiz_y = effectiveTurretAngle.getSin() * horiz + fieldSpeeds.vyMetersPerSecond;
        Translation3d initial = new Pose3d(swervePose)
            .plus(new Transform3d(-0.1651, 0.0, 0.367722, Rotation3d.kZero)).getTranslation();
        Translation3d[] trajectory = new Translation3d[20];
        for (int i = 0; i < 20; i++) {
            double time = i * (tof / 19.0);
            double z =
                Constants.Shooter.shooterHeight.in(Meters) + vert * time - 0.5 * 9.81 * time * time;
            double x = horiz_x * time;
            double y = horiz_y * time;
            trajectory[i] = initial.plus(new Translation3d(x, y, z));
        }
        Logger.recordOutput(name, trajectory);
    }

}
