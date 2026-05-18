package frc.gen;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import javax.swing.SwingUtilities;

public class TestApp {

    public static void main(String[] argv) {
        SwingUtilities.invokeLater(() -> {
            FrameProvider provider =
                new VideoFrameProvider("/home/watson/frc/2026/FRC2026/test.mp4");
            FieldLayout fieldLayout = new WPILibFieldLayout(
                AprilTagFields.kDefaultField, Units.inchesToMeters(6.5));
            PoseEstimator estimator = new PoseEstimator(fieldLayout);
            new VideoViewer(provider, estimator);
        });
    }

}
