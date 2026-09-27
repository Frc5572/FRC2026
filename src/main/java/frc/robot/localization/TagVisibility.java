package frc.robot.localization;

import java.util.function.IntPredicate;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * Predicts which AprilTags a camera should be able to see from a pose. Used to tell "no tags
 * because none are in view from here" apart from "no tags because the pose estimate is wrong".
 */
public final class TagVisibility {

    private TagVisibility() {}

    /**
     * Counts the field tags a camera would see from the given pose: in front of the camera, within
     * its horizontal field of view, within {@code maxDistance}, and facing the camera.
     *
     * @param robotPose robot pose on the field
     * @param robotToCamera camera mounting transform
     * @param horizontalFov full horizontal field of view
     * @param maxDistance furthest tag distance (meters) that still produces usable detections
     * @param maxIncidence largest angle between the tag's normal and the direction to the camera
     * @param tagFilter which tag IDs to count
     * @return number of tags expected to be visible
     */
    public static int countVisible(Pose2d robotPose, Transform3d robotToCamera,
        Rotation2d horizontalFov, double maxDistance, Rotation2d maxIncidence,
        IntPredicate tagFilter) {
        Pose3d cameraPose = new Pose3d(robotPose).plus(robotToCamera);
        Translation2d camera = cameraPose.getTranslation().toTranslation2d();
        double halfFov = horizontalFov.getRadians() / 2.0;
        int count = 0;
        for (AprilTag tag : Constants.Vision.fieldLayout.getTags()) {
            if (!tagFilter.test(tag.ID)) {
                continue;
            }
            var inCamera = tag.pose.relativeTo(cameraPose).getTranslation();
            if (inCamera.getX() <= 0 || inCamera.getNorm() > maxDistance
                || Math.abs(Math.atan2(inCamera.getY(), inCamera.getX())) > halfFov) {
                continue;
            }
            Translation2d toCamera = camera.minus(tag.pose.getTranslation().toTranslation2d());
            Rotation2d tagNormal = tag.pose.getRotation().toRotation2d();
            if (Math.abs(toCamera.getAngle().minus(tagNormal).getRadians()) > maxIncidence
                .getRadians()) {
                continue;
            }
            count++;
        }
        return count;
    }
}
