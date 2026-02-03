package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.SwerveState;
import frc.robot.util.Tuples.Tuple2;

/**
 * Vision subsystem responsible for processing camera-based pose observations and contributing
 * vision measurements to the drivetrain pose estimator.
 *
 * <p>
 * This subsystem acts as a bridge between one or more vision pipelines (e.g. PhotonVision) and the
 * {@link SwerveState} pose estimator.
 *
 * <h2>Timestamp handling</h2> Vision measurements are frequently delayed relative to the control
 * loop. This subsystem ensures measurements are applied in chronological order so that the pose
 * estimator can correctly replay history using its internal pose buffer.
 *
 * <h2>Multiple cameras</h2> The subsystem supports an arbitrary number of cameras, each with its
 * own calibration and transform defined in {@link Constants.Vision#cameraConstants}. All cameras
 * contribute measurements into a single shared pose estimate.
 *
 * <h2>Visualization</h2> For debugging and analysis, the subsystem publishes 3D visualization data
 * showing the relationship between the robot, cameras, and observed AprilTags. This data is
 * intended for tools such as AdvantageScope and is not used for control.
 */
@NullMarked
public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final VisionIO.CameraInputs[] cameraInputs;
    private final String[] cameraInputKeys;
    private final SwerveState state;
    private final Translation3d[][] cameraViz;
    private final String[] cameraVizKeys;
    private final boolean[] cameraContributed;
    private final String[] cameraContributedKeys;

    PhotonCamera camera = new PhotonCamera("camera");

    /**
     * Creates the vision subsystem.
     *
     * @param state shared swerve pose estimator to receive vision updates
     * @param io vision IO implementation responsible for acquiring camera results
     */
    public Vision(SwerveState state, VisionIO io) {
        super("Vision");
        this.io = io;
        this.state = state;
        this.cameraInputs = IntStream.range(0, Constants.Vision.cameraConstants.length)
            .mapToObj((_x) -> new VisionIO.CameraInputs()).toArray(VisionIO.CameraInputs[]::new);
        this.cameraInputKeys = IntStream.range(0, Constants.Vision.cameraConstants.length)
            .mapToObj((i) -> "Vision/Camera_" + i).toArray(String[]::new);
        this.cameraViz = IntStream.range(0, Constants.Vision.cameraConstants.length)
            .mapToObj((_x) -> new Translation3d[0]).toArray(Translation3d[][]::new);
        this.cameraVizKeys = IntStream.range(0, Constants.Vision.cameraConstants.length)
            .mapToObj((i) -> "Vision/AprilTagViz_" + i).toArray(String[]::new);
        this.cameraContributed = new boolean[Constants.Vision.cameraConstants.length];
        this.cameraContributedKeys = IntStream.range(0, Constants.Vision.cameraConstants.length)
            .mapToObj((i) -> "Vision/Contributed_" + i).toArray(String[]::new);
    }

    @Override
    public void periodic() {
        io.updateInputs(cameraInputs);
        for (int i = 0; i < cameraInputs.length; i++) {
            Logger.processInputs(cameraInputKeys[i], cameraInputs[i]);
        }

        // Get 2-tuples of (camera idx, result)
        List<Tuple2<Integer, PhotonPipelineResult>> results = new ArrayList<>();
        for (int i = 0; i < cameraInputs.length; i++) {

            for (int j = 0; j < cameraInputs[i].results.length; j++) {
                var result = cameraInputs[i].results[j];
                results.add(new Tuple2<>(i, result));
            }
        }

        // Sort by timestamp (earlier pipeline results come first)
        results.sort(
            (a, b) -> Double.compare(a._1().getTimestampSeconds(), b._1().getTimestampSeconds()));

        for (int i = 0; i < Constants.Vision.cameraConstants.length; i++) {
            cameraViz[i] = new Translation3d[0];
        }

        for (var result : results) {
            cameraContributed[result._0()] = state
                .addVisionObservation(Constants.Vision.cameraConstants[result._0()], result._1());
            for (int i = 0; i < result._1().targets.size(); i++) {
                var cameraPose = new Pose3d(state.getGlobalPoseEstimate())
                    .plus(Constants.Vision.cameraConstants[result._0()].robotToCamera);
                var target = result._1().targets.get(i);
                Constants.Vision.fieldLayout.getTagPose(target.fiducialId).ifPresent(pose -> {
                    cameraViz[result._0()] = addTwo(cameraViz[result._0()],
                        cameraPose.getTranslation(), pose.getTranslation());
                });
            }
        }

        for (int i = 0; i < Constants.Vision.cameraConstants.length; i++) {
            Logger.recordOutput(cameraContributedKeys[i], cameraContributed[i]);
            if (cameraViz[i].length == 0) {
                continue;
            }
            Logger.recordOutput(cameraVizKeys[i], cameraViz[i]);
        }
    }

    private Translation3d[] addTwo(Translation3d[] translations, Translation3d newTranslation1,
        Translation3d newTranslation2) {
        Translation3d[] res = new Translation3d[translations.length + 2];
        System.arraycopy(translations, 0, res, 0, translations.length);
        res[translations.length] = newTranslation1;
        res[translations.length + 1] = newTranslation2;
        return res;
    }

    /**
     * 
     * @return returns a boolean value based on whether or not the robot sees the correct april tags
     *         (when the robot is close to the trench)
     */
    public boolean seesTrenchTags() {
        List<Integer> trenchTags = new ArrayList<>();
        trenchTags.add(1);
        trenchTags.add(12);
        trenchTags.add(7);
        trenchTags.add(6);
        trenchTags.add(17);
        trenchTags.add(28);
        trenchTags.add(22);
        trenchTags.add(23);
        boolean seesTags = false;
        for (var input : cameraInputs) {
            for (var results : input.results) {
                var result = camera.getLatestResult();
                if (results.hasTargets()) {
                    int tagID = result.getBestTarget().getFiducialId();
                    if (trenchTags.contains(tagID)) {
                        seesTags = true;

                    }
                }
            }
        }
        return seesTags;
    }

}
