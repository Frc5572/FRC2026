package frc.robot.viz;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.Arrays;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.sim.SimulatedRobotState;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;

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

    private static final Translation3d hoodRotationCenter =
        new Translation3d(-0.052975, 0, 0.490041);
    private static final Translation3d intakeRotationCenter =
        new Translation3d(0.149203, 0, 0.245623);
    private static final Translation3d climberRotationCenter =
        new Translation3d(-0.258763, 0, 0.198437);
    private static final Distance hooksDown = Meters.of(0.5);


    private final Pose3d[] gtState;
    private final Pose3d[] estState = new Pose3d[numPoses];

    private final SimulatedRobotState sim;
    private final Swerve swerve;
    private final Turret turret;
    private final AdjustableHood hood;
    private final Intake intake;
    private final Climber climber;
    private final Shooter shooter;

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
        this.sim = sim;
        this.swerve = swerve;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
        this.climber = climber;
        this.shooter = shooter;
        if (sim != null) {
            gtState = new Pose3d[numPoses];
        } else {
            gtState = estState;
        }
    }



    /**
     * Publishes visualization data for the current control loop iteration.
     *
     * <p>
     * This method should be called periodically (e.g., from {@code robotPeriodic}).
     */
    public void periodic() {
        Pose3d robotPose = new Pose3d(swerve.state.getGlobalPoseEstimate());
        Logger.recordOutput("Viz/EstPose", robotPose);
        update(estState, turret.getTurretHeading(), hood.inputs.relativeAngle,
            climber.inputs.positionPivot, climber.inputs.positionTelescope,
            intake.inputs.rightHopperPosition, Arrays.stream(swerve.modules)
                .map(mod -> mod.inputs.anglePosition).toArray(Rotation2d[]::new));
        Logger.recordOutput("Viz/EstState", estState);
        Pose3d turretCenter = new Pose3d(swerve.state.getGlobalPoseEstimate())
            .plus(new Transform3d(Constants.Vision.turretCenter.getTranslation(),
                new Rotation3d(turret.getTurretHeading())));
        for (var camera : Constants.Vision.cameraConstants) {
            Pose3d pose = robotPose.plus(camera.robotToCamera);
            if (camera.isTurret) {
                pose = turretCenter.plus(camera.robotToCamera);
            }
            Logger.recordOutput("Viz/Cameras/" + camera.name + "/Est", pose);
        }
        if (sim != null) {
            Logger.recordOutput("Viz/ActualPose", sim.getGroundTruthPose());
            update(gtState, Rotation2d.fromRadians(sim.turret.turretTarget),
                Radians.of(sim.adjustableHood.hood.position), climber.inputs.positionPivot,
                climber.inputs.positionTelescope, intake.inputs.rightHopperPosition,
                Arrays.stream(sim.swerveDrive.mapleSim.getModules())
                    .map(mod -> mod.getSteerAbsoluteFacing()).toArray(Rotation2d[]::new));
            Logger.recordOutput("Viz/ActualState", gtState);
            for (var camera : sim.visionSim.staticCameras.entrySet()) {
                Pose3d pose = sim.visionSim.visionSim.getCameraPose(camera.getValue()).get();
                Logger.recordOutput("Viz/Cameras/" + camera.getKey() + "/Actual", pose);
            }
            for (var camera : sim.visionSim.turretCameras.entrySet()) {
                Pose3d pose = sim.visionSim.turretVisionSim.getCameraPose(camera.getValue()).get();
                Logger.recordOutput("Viz/Cameras/" + camera.getKey() + "/Actual", pose);
            }
        }
    }

    private void update(Pose3d[] out, Rotation2d turretAngle, Angle hoodAngle, Angle climberAngle,
        Distance climberHeight, Distance intakeOut, Rotation2d[] modules) {
        out[hoodIndex] = new Pose3d()
            .rotateAround(hoodRotationCenter, new Rotation3d(0, hoodAngle.in(Radians), 0))
            .rotateAround(Constants.Vision.turretCenter.getTranslation(),
                new Rotation3d(0, 0, turretAngle.getRadians()));
        out[turretIndex] = new Pose3d().rotateAround(Constants.Vision.turretCenter.getTranslation(),
            new Rotation3d(0, 0, turretAngle.getRadians()));

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
    }

}
