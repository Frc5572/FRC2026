package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Milliseconds;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.jspecify.annotations.NullMarked;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSim;

/** Simulation of vision using built-in PhotonVision simulator. */
@NullMarked
public class VisionSim extends VisionReal {

    private final SwerveDriveSimulation sim;
    private final VisionSystemSim visionSim;

    /** Simulation of vision using built-in PhotonVision simulator. */
    public VisionSim(SwerveSim sim) {
        this.sim = sim.mapleSim;
        this.visionSim = new VisionSystemSim("main");

        visionSim.addAprilTags(Constants.Vision.fieldLayout);

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
            visionSim.addCamera(cameraSim, constants[i].robotToCamera);
        }
    }

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        visionSim.update(sim.getSimulatedDriveTrainPose());
        super.updateInputs(inputs);
    }

}
