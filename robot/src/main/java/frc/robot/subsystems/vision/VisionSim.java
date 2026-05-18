package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Milliseconds;
import java.util.HashMap;
import java.util.Map;
import org.jspecify.annotations.NullMarked;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

/** Simulation of vision using built-in PhotonVision simulator. */
@NullMarked
public class VisionSim extends VisionReal {

    public final VisionSystemSim visionSim;
    public final VisionSystemSim turretVisionSim;
    public final Map<String, PhotonCameraSim> staticCameras = new HashMap<>();
    public final Map<String, PhotonCameraSim> turretCameras = new HashMap<>();

    /** Simulation of vision using built-in PhotonVision simulator. */
    public VisionSim() {
        this.visionSim = new VisionSystemSim("main");
        this.turretVisionSim = new VisionSystemSim("turret");

        visionSim.addAprilTags(Constants.Vision.fieldLayout);
        turretVisionSim.addAprilTags(Constants.Vision.fieldLayout);

        var constants = Constants.Vision.cameraConstants;
        for (int i = 0; i < constants.length; i++) {
            SimCameraProperties props = new SimCameraProperties();
            props.setCalibration(constants[i].width, constants[i].height,
                constants[i].horizontalFieldOfView);
            props.setCalibError(constants[i].calibrationErrorMean,
                constants[i].calibrationErrorStdDev);
            props.setFPS(constants[i].simFps.in(Hertz));
            props.setAvgLatencyMs(constants[i].simLatency.in(Milliseconds));
            props.setLatencyStdDevMs(constants[i].simLatencyStdDev.in(Milliseconds));
            PhotonCameraSim cameraSim = new PhotonCameraSim(this.cameras[i], props);
            if (constants[i].isTurret) {
                turretVisionSim.addCamera(cameraSim, constants[i].robotToCamera);
                turretCameras.put(constants[i].name, cameraSim);
            } else {
                visionSim.addCamera(cameraSim, constants[i].robotToCamera);
                staticCameras.put(constants[i].name, cameraSim);
            }
        }
    }

    /** Update camera poses using robot state. */
    public void updateState(Pose3d robotPose, Angle turretAngle) {
        visionSim.update(robotPose);
        // In the future, should update based on turret state.
        turretVisionSim
            .update(robotPose.plus(new Transform3d(Constants.Vision.turretCenter.getTranslation(),
                new Rotation3d(new Rotation2d(turretAngle)))));
    }

}
