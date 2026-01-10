package frc.robot.subsystems.vision;

import java.util.Arrays;
import org.jspecify.annotations.NullMarked;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants;

/** PhotonVision-attached implementation */
@NullMarked
public class VisionReal implements VisionIO {

    protected final PhotonCamera[] cameras;

    /** PhotonVision-attached implementation */
    public VisionReal() {
        cameras = Arrays.stream(Constants.Vision.cameraConstants)
            .map((consts) -> new PhotonCamera(consts.name)).toArray(PhotonCamera[]::new);
    }

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        for (int i = 0; i < cameras.length; i++) {
            inputs[i].results =
                cameras[i].getAllUnreadResults().toArray(PhotonPipelineResult[]::new);
            inputs[i].cameraMatrix = cameras[i].getCameraMatrix();
            inputs[i].distCoeffs = cameras[i].getDistCoeffs();
        }
    }

}
