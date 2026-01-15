package frc.robot.subsystems.vision.color;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

/** Color detector hardware layer */
public class ColorDetectionReal implements ColorDetectionIO {

    private final DoubleSubscriber yaw;
    private final BooleanSubscriber seesYellow;
    private final NetworkTableInstance inst;

    public ColorDetectionReal() {
        inst = NetworkTableInstance.getDefault();

        inst.startClient4("robot");

        DoubleTopic yawTopic = inst.getTable("datatable").getDoubleTopic("yaw");
        BooleanTopic seesYellowTopic = inst.getTable("datatable").getBooleanTopic("seesYellow");

        yaw = yawTopic.subscribe(0.0);
        seesYellow = seesYellowTopic.subscribe(false, PubSubOption.periodic(1));
    }

    public double getYaw() {
        return yaw.get();
    }

    @Override
    public void updateInputs(ColorInputs inputs) {
        inputs.yaw = Radians.of(getYaw());
        inputs.seesYellow = seesYellow.get();
    }
}
