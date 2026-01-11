package frc.robot.subsystems.vision.colorDetection;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ColorReal implements ColorIO {

    private final DoubleSubscriber yaw;
    private final NetworkTableInstance inst;

    public ColorReal() {
        inst = NetworkTableInstance.getDefault();

        inst.startClient4("robot");

        DoubleTopic yawTopic = inst.getTable("datatable").getDoubleTopic("yaw");

        yaw = yawTopic.subscribe(0.0);
    }

    public double getYaw() {
        return yaw.get();
    }

    @Override
    public void updateInputs(ColorInputs inputs) {
        inputs.yaw = Radians.of(getYaw());
    }
}
