package frc.robot.localization;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.localization.utils.RejectionReason;
import frc.robot.localization.utils.Result;
import frc.robot.subsystems.vision.CameraConstants;

public class CameraProcessor {
    private final CameraConstants cameraConstants;

    public CameraProcessor(CameraConstants cameraConstants) {
        this.cameraConstants = cameraConstants;
    }

    public Result<VisionObservation, RejectionReason> process(PhotonPipelineResult result,
        ChassisSpeeds currentSpeeds) {
        var multiTag = result.getMultiTagResult();
        Transform3d robotToCamera_ = cameraConstants.robotToCamera;
        double translationSpeed =
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double rotationSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);
        if (cameraConstants.isTurret) {
            var maybeTurretRotation = currentTurretAngle.getSample(result.getTimestampSeconds());
            // var maybeTurretRotationM1 =
            // currentTurretAngle.getSample(result.getTimestampSeconds() - 0.1);
            // var maybeTurretRotationP1 =
            // currentTurretAngle.getSample(result.getTimestampSeconds() + 0.1);
            if (maybeTurretRotation.isEmpty()) {
                return false;
            }
            // if (Math.abs(angleDiff(maybeTurretRotationM1.get(), maybeTurretRotationP1.get())
            // .in(Degrees)) > 5) {
            // return false;
            // }
            robotToCamera_ = getTurretRobotToCamera(robotToCamera_, maybeTurretRotation.get());
        }
        if (!initted) {
            final Transform3d robotToCamera = robotToCamera_;
            multiTag.ifPresent(multiTag_ -> {
                Transform3d best = multiTag_.estimatedPose.best;
                Pose3d cameraPose =
                    new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
                Pose3d robotPose = cameraPose.plus(robotToCamera.inverse());
                // reading - gyroOffset = actual
                // gyroOffset = reading - actual
                gyroOffset = prevGyroReading.minus(robotPose.toPose2d().getRotation());
                visionAdjustedOdometry.resetPose(robotPose.toPose2d());
                Logger.recordOutput("State/initPose", getGlobalPoseEstimate());
                currentTurretAngle.clear();
                initted = true;
            });
            return initted;
        } else {
            double velocityStdDev = cameraConstants.simLatencyStdDev.in(Seconds);
            double velocityTranslationError = translationSpeed * velocityStdDev;
            double velocityRotationError = rotationSpeed * velocityStdDev;
            Logger.recordOutput("State/velocityTranslationError", velocityTranslationError);
            Logger.recordOutput("State/velocityRotationError", velocityRotationError);

            var bestTarget = result.hasTargets() ? result.getBestTarget() : null;
            if (bestTarget == null) {
                return false;
            }
            var bestTagPose = Constants.Vision.fieldLayout.getTagPose(bestTarget.getFiducialId());
            if (bestTagPose.isEmpty()) {
                return false;
            }

            if (multiTag.isPresent()) {
                // Multi Tag
                Transform3d best = multiTag.get().estimatedPose.best;
                Pose3d cameraPose =
                    new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
                Logger.recordOutput("State/Camera/" + cameraConstants.name + "/cameraPose",
                    cameraPose);
                Logger.recordOutput("State/Camera/" + cameraConstants.name + "/correctedCameraPose",
                    cameraPose);
                Pose3d estRobotPose = cameraPose.plus(robotToCamera_.inverse());
                Logger.recordOutput("State/Camera/" + cameraConstants.name + "/estRobotPose",
                    estRobotPose);
                double stdDevMultiplier = stdDevMultiplier(result.targets, cameraPose);
                double translationStdDev =
                    stdDevMultiplier * velocityTranslationError + cameraConstants.translationError;
                double rotationStdDev =
                    stdDevMultiplier * velocityRotationError + cameraConstants.rotationError;
                if (cameraConstants.isTurret) {
                    boolean isStationary = this.lastTimeMoved + 0.5 < result.getTimestampSeconds();
                    Logger.recordOutput("State/Camera/" + cameraConstants.name + "/isStationary",
                        isStationary);
                    Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stationaryValue",
                        this.lastTimeMoved - result.getTimestampSeconds());
                    Logger.recordOutput("State/Camera/" + cameraConstants.name + "/lastMoved",
                        this.lastTimeMoved);
                    Logger.recordOutput("State/Camera/" + cameraConstants.name + "/timestamp",
                        result.getTimestampSeconds());
                    if (isStationary || (RobotBase.isReal()
                        && FieldConstants.isOnBump(getGlobalPoseEstimate()))) {
                        var estRobotPose2d = estRobotPose.toPose2d();
                        if (estRobotPose2d.getTranslation()
                            .getSquaredDistance(getGlobalPoseEstimate().getTranslation()) > Math
                                .pow(Units.inchesToMeters(3), 2)) {
                            visionAdjustedOdometry
                                .resetTranslation(estRobotPose2d.getTranslation());
                        }
                    }
                    if (!isStationary) {
                        rotationStdDev = 10000.0;
                    }
                }
                Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevMultipler",
                    stdDevMultiplier);
                Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevTranslation",
                    translationStdDev);
                Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevRotation",
                    rotationStdDev);
                if (cameraConstants.findConstants) {
                    var transform =
                        new Transform3d(new Pose3d(getGlobalPoseEstimate()), cameraPose);
                    Logger.recordOutput(
                        "State/Camera/" + cameraConstants.name + "/foundConstant/translation",
                        transform.getTranslation());
                    Logger.recordOutput(
                        "State/Camera/" + cameraConstants.name + "/foundConstant/rotation/x",
                        Units.radiansToDegrees(transform.getRotation().getX()));
                    Logger.recordOutput(
                        "State/Camera/" + cameraConstants.name + "/foundConstant/rotation/y",
                        Units.radiansToDegrees(transform.getRotation().getY()));
                    Logger.recordOutput(
                        "State/Camera/" + cameraConstants.name + "/foundConstant/rotation/z",
                        Units.radiansToDegrees(transform.getRotation().getZ()));
                    return false;
                }
                VisionObservation observations = new VisionObservation(cameraPose, robotToCamera_,
                    translationStdDev, rotationStdDev, result.getTimestampSeconds());
                return Result<observations>;
            }
        }
        return false;
    }
}
