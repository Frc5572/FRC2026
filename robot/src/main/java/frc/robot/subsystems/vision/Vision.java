package frc.robot.subsystems.vision;

import java.util.List;
import frc.robot.localization.DrivetrainState;

public class Vision {

    private final VisionIO io;
    private final DrivetrainState drivetrainState;
    private final List<CameraProcessor> processors;

    public Vision(VisionIO io, DrivetrainState drivetrainState, List<CameraProcessor> processors) {
        this.io = io;
        this.drivetrainState = drivetrainState;
        this.processors = processors;
    }

}
