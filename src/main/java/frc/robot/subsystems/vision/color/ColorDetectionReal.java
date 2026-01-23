package frc.robot.subsystems.vision.color;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;

/** Color detector hardware layer */
public class ColorDetectionReal implements ColorDetectionIO {

    private final DoubleSubscriber yaw;
    private final BooleanSubscriber seesYellow;
    private final NetworkTableInstance inst;
    private final StringSubscriber error;

    /** class constructor */
    public ColorDetectionReal() {
        inst = NetworkTableInstance.getDefault();

        DoubleTopic yawTopic = inst.getTable("ColorPI").getDoubleTopic("yaw");
        BooleanTopic seesYellowTopic = inst.getTable("ColorPI").getBooleanTopic("seesYellow");
        StringTopic errorTopic = inst.getTable("ColorPI").getStringTopic("error");

        yaw = yawTopic.subscribe(0.0);
        seesYellow = seesYellowTopic.subscribe(false, PubSubOption.periodic(0.1));
        error = errorTopic.subscribe("", PubSubOption.periodic(0.1));
    }

    public double getYaw() {
        return yaw.get();
    }

    @Override
    public void updateInputs(ColorInputs inputs) {
        inputs.yaw = Radians.of(getYaw());
        inputs.seesYellow = seesYellow.get();
        inputs.error = error.get();
    }
}
