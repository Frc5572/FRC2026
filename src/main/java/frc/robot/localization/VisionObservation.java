package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Vision Observation Record
 *
 * @param isTurret whether the observation came from a turret-mounted camera, whose rotation is only
 *        trustworthy while the robot and turret are stationary
 */
public record VisionObservation(Pose3d cameraPose, Transform3d robotToCamera,
    double translationStdDev, double rotationStdDev, double timestamp, boolean isTurret) {

    public Vector<N3> getStdDev() {
        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }
}
