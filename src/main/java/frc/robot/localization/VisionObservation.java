package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

/** Vision Observation Record */
public record VisionObservation(Pose3d cameraPose, Transform3d robotToCamera,
    double translationStdDev, double rotationStdDev, double timestamp, boolean isTurret,
    String cameraName) {

    public VisionObservation(Pose3d cameraPose, Transform3d robotToCamera,
        double translationStdDev, double rotationStdDev, double timestamp) {
        this(cameraPose, robotToCamera, translationStdDev, rotationStdDev, timestamp, false, "");
    }

    public VisionObservation(Pose3d cameraPose, Transform3d robotToCamera,
        double translationStdDev, double rotationStdDev, double timestamp, boolean isTurret) {
        this(cameraPose, robotToCamera, translationStdDev, rotationStdDev, timestamp, isTurret, "");
    }

    public Vector<N3> getStdDev() {
        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }
}
