package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Milliseconds;
import org.jspecify.annotations.NullMarked;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Constants;
import frc.robot.sim.SimulatedRobotState;

/** Simulation of vision using built-in PhotonVision simulator. */
@NullMarked
public class VisionSim extends VisionReal {

    private final SimulatedRobotState sim;
    private final VisionSystemSim visionSim;
    private final VisionSystemSim turretVisionSim;

    /** Simulation of vision using built-in PhotonVision simulator. */
    public VisionSim(SimulatedRobotState sim) {
        this.sim = sim;
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
            (constants[i].isTurret ? turretVisionSim : visionSim).addCamera(cameraSim,
                constants[i].robotToCamera);
        }
    }

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        visionSim.update(sim.swerveDrive.mapleSim.getSimulatedDriveTrainPose());
        // In the future, should update based on turret state.
        turretVisionSim.update(sim.swerveDrive.mapleSim.getSimulatedDriveTrainPose());
        super.updateInputs(inputs);
    }

}
