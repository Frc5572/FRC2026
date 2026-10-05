package frc.robot.localization;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.math.geometry.Penetration;
import frc.robot.math.geometry.Rectangle;
import frc.robot.math.geometry.SeparatingAxis;
import frc.robot.subsystems.vision.CameraConstants;

/**
 * Filters a {@link org.photonvision.targeting.PhotonPipelineResult} and, when the result passes all
 * quality checks, produces a {@link VisionObservation} ready for fusion into
 * {@link DrivetrainState}.
 *
 * <p>
 * This class is stateless with respect to the robot pose it only decides whether a camera frame is
 * trustworthy and computes the appropriate standard deviations. Pose-estimator mutations remain the
 * exclusive responsibility of {@link DrivetrainState}.
 *
 * <p>
 * For fixed cameras, use the single-argument constructor. For turreted cameras, supply a
 * {@link TurretCameraAdapter} that is kept up to date with turret angle samples by the
 * {@code Turret} subsystem.
 */
public class CameraProcessor {

    private final CameraConstants cameraConstants;

    private final Rectangle robotRect = new Rectangle("pose", Pose2d.kZero,
        Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);
    private final Rectangle[] hubRects = new Rectangle[] {
        new Rectangle("hub", new Pose2d(FieldConstants.Hub.centerHub, Rotation2d.kZero),
            FieldConstants.Hub.width, FieldConstants.Hub.width),
        new Rectangle("oppHub",
            new Pose2d(FieldConstants.fieldLength - FieldConstants.Hub.centerHub.getX(),
                FieldConstants.Hub.centerHub.getY(), Rotation2d.kZero),
            FieldConstants.Hub.width, FieldConstants.Hub.width)};
    private final Penetration hubPenetration = new Penetration("HubPen");

    /**
     * Adapter that resolves the time-varying robot-to-camera transform for a turreted camera.
     * {@code null} for fixed (non-turret) cameras.
     */
    @Nullable
    private final TurretCameraAdapter adapter;

    /**
     * Creates a {@code CameraProcessor} for a fixed (non-turret) camera.
     *
     * @param cameraConstants tuning constants for this camera
     */
    public CameraProcessor(CameraConstants cameraConstants) {
        this(cameraConstants, null);
    }

    /**
     * Creates a {@code CameraProcessor}, optionally backed by a {@link TurretCameraAdapter} for
     * turreted cameras.
     *
     * @param cameraConstants tuning constants for this camera
     * @param adapter turret adapter, or {@code null} if this is a fixed camera
     */
    public CameraProcessor(CameraConstants cameraConstants, @Nullable TurretCameraAdapter adapter) {
        this.cameraConstants = cameraConstants;
        this.adapter = adapter;
    }


    /**
     * processes photon results to get robots pose
     *
     * @param result photon results
     * @param currentSpeeds robot speeds
     * @return position data
     */
    public Result<VisionObservation, RejectionReason> process(PhotonPipelineResult result,
        ChassisSpeeds currentSpeeds) {
        return process(result, currentSpeeds, true);
    }

    /**
     * processes photon results to get robots pose
     *
     * @param result photon results
     * @param currentSpeeds robot speeds
     * @param isReal whether the vision observation is from real hardware or replay
     * @return position data
     */
    public Result<VisionObservation, RejectionReason> process(PhotonPipelineResult result,
        ChassisSpeeds currentSpeeds, boolean isReal) {
        var multiTag = result.getMultiTagResult();
        Transform3d robotToCamera_ = cameraConstants.robotToCamera;

        double translationSpeed =
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double rotationSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);

        if (adapter != null) {
            var maybeRobotToCamera =
                adapter.getRobotToCameraAt(robotToCamera_, result.getTimestampSeconds());
            if (maybeRobotToCamera.isEmpty()) {
                return Result.err(RejectionReason.MISSING_TURRET_ANGLE);
            }
            robotToCamera_ = maybeRobotToCamera.get();
        }

        double velocityStdDev = cameraConstants.simLatencyStdDev.in(Seconds);
        double velocityTranslationError = translationSpeed * velocityStdDev;
        double velocityRotationError = rotationSpeed * velocityStdDev;
        Logger.recordOutput("State/velocityTranslationError", velocityTranslationError);
        Logger.recordOutput("State/velocityRotationError", velocityRotationError);

        var bestTarget = result.hasTargets() ? result.getBestTarget() : null;
        if (bestTarget == null) {
            return Result.err(RejectionReason.NO_TARGETS);
        }

        var bestTagPose = Constants.Vision.fieldLayout.getTagPose(bestTarget.getFiducialId());
        if (bestTagPose.isEmpty()) {
            return Result.err(RejectionReason.NO_TARGETS);
        }

        if (multiTag.isPresent()) {
            Transform3d best = multiTag.get().estimatedPose.best;
            Pose3d cameraPose =
                new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());

            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/cameraPose", cameraPose);
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/correctedCameraPose",
                cameraPose);

            Pose3d estRobotPose = cameraPose.plus(robotToCamera_.inverse());
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/estRobotPose",
                estRobotPose);

            var estRobotPose2d = estRobotPose.toPose2d();
            robotRect.setPose(estRobotPose2d);

            // Reject if whole robot (including bumpers) extends outside field perimeter
            for (var corner : robotRect.getCorners()) {
                if (corner.getX() < 0.0 || corner.getX() > FieldConstants.fieldLength
                    || corner.getY() < 0.0 || corner.getY() > FieldConstants.fieldWidth) {
                    return Result.err(RejectionReason.OUTSIDE_FIELD);
                }
            }

            // Reject if robot penetrates either hub
            for (var hub : hubRects) {
                if (SeparatingAxis.solve(robotRect, hub, hubPenetration)) {
                    return Result.err(RejectionReason.INSIDE_HUB);
                }
            }

            double stdDevMultiplier = stdDevMultiplier(result.targets, cameraPose);
            double translationStdDev =
                stdDevMultiplier * velocityTranslationError + cameraConstants.translationError;
            double rotationStdDev =
                stdDevMultiplier * velocityRotationError + cameraConstants.rotationError;

            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevMultipler",
                stdDevMultiplier);
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevTranslation",
                translationStdDev);
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevRotation",
                rotationStdDev);

            if (cameraConstants.findConstants) {
                return Result.err(RejectionReason.NO_TARGETS);
            }

            return Result.ok(new VisionObservation(cameraPose, robotToCamera_, translationStdDev,
                rotationStdDev, result.getTimestampSeconds(), cameraConstants.isTurret,
                cameraConstants.name, isReal));
        }

        return Result.err(RejectionReason.SINGLE_TAG_ONLY);
    }


    private static double stdDevMultiplier(List<PhotonTrackedTarget> targets, Pose3d cameraPose) {
        double totalDistance = 0.0;
        int count = 0;
        for (var tag : targets) {
            var maybeTagPose = Constants.Vision.fieldLayout.getTagPose(tag.getFiducialId());
            if (maybeTagPose.isPresent()) {
                var tagPose = maybeTagPose.get();
                totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                count++;
            }
        }
        double avgDistance = totalDistance / count;
        double stddev = Math.pow(avgDistance, 2.0) / count;
        return stddev;
    }

    /** results tuple */
    public sealed interface Result<T, E> permits Ok, Err {
        public static <T, E> Result<T, E> ok(T value) {
            return new Ok<>(value);
        }

        public static <T, E> Result<T, E> err(E error) {
            return new Err<>(error);
        }
    }

    /** Ok results */
    public record Ok<T, E>(T value) implements Result<T, E> {
    }

    /** Err results */
    public record Err<T, E>(E error) implements Result<T, E> {
    }
}
