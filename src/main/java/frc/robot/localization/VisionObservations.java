package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

/** Vision Observations Record */
public record VisionObservations(Pose3d cameraPose, Transform3d robotToCamera,
    double translationStdDev, double rotationStdDev, double timestamp) {

    public Vector<N3> getStdDev() {
        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }
}
