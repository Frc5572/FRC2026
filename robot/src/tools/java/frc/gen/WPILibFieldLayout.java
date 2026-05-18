package frc.gen;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

public class WPILibFieldLayout implements FieldLayout {

    private final AprilTagFieldLayout layout;
    private final double halfSize;

    public WPILibFieldLayout(AprilTagFields field, double tagSizeMeters) {
        this.layout = AprilTagFieldLayout.loadField(field);
        this.halfSize = tagSizeMeters / 2.0;
    }

    @Override
    public double[][] getTagCorners(int tagId) {
        Optional<Pose3d> opt = layout.getTagPose(tagId);
        if (opt.isEmpty()) return null;

        Pose3d pose = opt.get();

        // Tag-local corners in WPILib NWU convention (tag faces along +X, Y=left, Z=up).
        // Order matches ArUco: top-left, top-right, bottom-right, bottom-left when viewed from front.
        Translation3d[] local = {
            new Translation3d(0, +halfSize, +halfSize),
            new Translation3d(0, -halfSize, +halfSize),
            new Translation3d(0, -halfSize, -halfSize),
            new Translation3d(0, +halfSize, -halfSize),
        };

        double[][] corners = new double[4][3];
        for (int i = 0; i < 4; i++) {
            Translation3d f = pose.getTranslation().plus(local[i].rotateBy(pose.getRotation()));
            corners[i][0] = f.getX();
            corners[i][1] = f.getY();
            corners[i][2] = f.getZ();
        }
        return corners;
    }
}
