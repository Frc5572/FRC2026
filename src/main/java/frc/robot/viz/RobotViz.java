package frc.robot.viz;

import java.util.Arrays;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.CameraConstants;

/**
 * Centralized visualization helper for publishing robot state to logging and visualization tools.
 *
 * <p>
 * Currently, this class publishes visualization data related to drivetrain state, including the
 * robot's global pose and derived camera poses.
 *
 * <p>
 * <b>Future intent:</b> This class is intended to become the single location responsible for
 * publishing visualization data for <em>all</em> robot subsystems, such as elevators, arms,
 * intakes, and other articulated mechanisms. Each subsystem will contribute geometry or pose data
 * that can be rendered together in a unified visualization view.
 *
 * <p>
 * This class supports real operation:
 * <ul>
 * <li>Certain entries with keys indicating ground truth use estimates, often duplicating other
 * entries (e.g. {@code ActualPose} will be equivalent to {@code GlobalEstPose}).</li>
 * </ul>
 *
 * <p>
 * All outputs are published via {@link Logger} and are intended strictly for debugging, analysis,
 * and visualization (e.g., AdvantageScope). No control or decision-making logic should depend on
 * this class.
 */
@NullMarked
public class RobotViz {

    private final @Nullable Supplier<Pose3d> robotPoseSupplier;
    private final Supplier<Pose3d> estPoseSupplier;

    /**
     * Creates a new visualization helper.
     * 
     * @param swerve live swerve subsystem providing pose estimates when not simulating
     */
    public RobotViz(Swerve swerve) {
        estPoseSupplier = () -> new Pose3d(swerve.state.getGlobalPoseEstimate());
        robotPoseSupplier = estPoseSupplier;

    }

    /**
     * Publishes visualization data for the current control loop iteration.
     *
     * <p>
     * This method should be called periodically (e.g., from {@code robotPeriodic}).
     *
     * <p>
     * As additional subsystems are integrated, this method will be extended to publish poses,
     * transforms, and geometry for other mechanisms relative to the robot frame.
     */
    public void periodic() {
        Pose3d robotPose = robotPoseSupplier.get();
        Pose3d estPose = estPoseSupplier.get();
        Logger.recordOutput("Viz/ActualPose", robotPose);
        for (CameraConstants constants : Constants.Vision.cameraConstants) {
            Logger.recordOutput("Viz/Cameras/" + constants.name + "/ActualPose",
                robotPose.plus(constants.robotToCamera));
        }
        Logger.recordOutput("Viz/GlobalEstPose", estPose);
        Logger.recordOutput("Viz/CameraPoses", Arrays.stream(Constants.Vision.cameraConstants)
            .map(consts -> robotPose.plus(consts.robotToCamera)).toArray(Pose3d[]::new));
    }

}
