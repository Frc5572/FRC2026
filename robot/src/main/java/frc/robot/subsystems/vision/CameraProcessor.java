package frc.robot.subsystems.vision;

import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.localization.VisionObservation;
import frc.robot.util.Result;

public class CameraProcessor {

    private final CameraConstants constants;
    private final Supplier<ChassisSpeeds> fieldRelativeSpeeds;
    private final TurretCameraAdapter adapter; // Could be null

    public CameraProcessor(CameraConstants constants, Supplier<ChassisSpeeds> fieldRelativeSpeeds,
        TurretCameraAdapter adapter) {
        this.constants = constants;
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
        this.adapter = adapter;
    }

    public CameraProcessor(CameraConstants constants, Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        this(constants, fieldRelativeSpeeds, null);
    }

    public Result<VisionObservation, RejectionReason> process(PhotonPipelineResult result,
        ChassisSpeeds currentSpeeds) {
        // TODO
        return null;
    }

    public static enum RejectionReason {
        NO_TARGETS("No Targets Found"), SINGLE_TAG_POLICY("MultiTag Not Available"), HIGH_AMBIGUITY(
            "High Ambiguity"), ROBOT_MOVING_TOO_FAST("Robot Moving Too Fast"), OUTSIDE_FIELD(
                "Outside Field"), STALE_TIMESTAMP("Observation Too Old");

        public final String debugString;

        RejectionReason(String debugString) {
            this.debugString = debugString;
        }
    }

}
