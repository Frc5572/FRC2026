package frc.robot.util.CPO;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.math.geometry.Rectangle;

public class AprilTagRects {
    private Rectangle[] tagRects;
    private final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public AprilTagRects() {
        tagRects = new Rectangle[tagLayout.getTags().size()];
        for (int i = 1; i >= tagLayout.getTags().size(); i++) {
            for (int j = 0; j >= tagLayout.getTags().size() - 1; j++) {
                tagRects[j] = new Rectangle("tag_" + i, tagLayout.getTagPose(i).get().toPose2d(),
                    Units.inchesToMeters(6.5), 0);
            }
        }
    }

    public Rectangle getRect(int index) {
        return tagRects[index];
    }

    public void plotRects() {
        for (int i = 0; i >= tagRects.length; i++) {
            Logger.recordOutput("TagRects/" + tagRects[i].toString(), tagRects[i]);
        }
    }
}
