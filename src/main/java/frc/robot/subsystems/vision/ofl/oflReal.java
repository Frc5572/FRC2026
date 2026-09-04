package frc.robot.subsystems.vision.ofl;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class oflReal implements oflIO {

    private final DoubleSubscriber xMeters;
    private final DoubleSubscriber yMeters;
    private final DoubleSubscriber thetaDeg;

    private final DoubleSubscriber deltaXMeters;
    private final DoubleSubscriber deltaYMeters;
    private final DoubleSubscriber deltaThetaDeg;

    private final IntegerSubscriber inliers;
    private final DoubleSubscriber confidence;
    private final BooleanSubscriber valid;

    private final IntegerSubscriber timestampUs;
    private final BooleanSubscriber jetsonConnected;

    public oflReal() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        NetworkTable table = ntInstance.getTable("VisualOdometry");

        xMeters = table.getDoubleTopic("xMeters").subscribe(0.0);
        yMeters = table.getDoubleTopic("yMeters").subscribe(0.0);
        thetaDeg = table.getDoubleTopic("thetaDeg").subscribe(0.0);

        deltaXMeters = table.getDoubleTopic("deltaXMeters").subscribe(0.0);
        deltaYMeters = table.getDoubleTopic("deltaYMeters").subscribe(0.0);
        deltaThetaDeg = table.getDoubleTopic("deltaThetaDeg").subscribe(0.0);


        inliers = table.getIntegerTopic("inliers").subscribe(0);
        confidence = table.getDoubleTopic("confidence").subscribe(0.0);
        valid = table.getBooleanTopic("valid").subscribe(false);

        timestampUs = table.getIntegerTopic("timestampUs").subscribe(0);
        jetsonConnected = table.getBooleanTopic("jetsonConnected").subscribe(false);
    }

    @Override
    public void updateInputs(oflInputs inputs) {

        // Pose
        inputs.xMeters = xMeters.get();
        inputs.yMeters = yMeters.get();
        inputs.thetaDeg = thetaDeg.get();

        // Delta
        inputs.deltaXMeters = deltaXMeters.get();
        inputs.deltaYMeters = deltaYMeters.get();
        inputs.deltaThetaDeg = deltaThetaDeg.get();

        // Quality
        inputs.inliers = (int) inliers.get();
        inputs.confidence = confidence.get();
        inputs.valid = valid.get();

        // Status
        inputs.timestampUs = timestampUs.get();
        inputs.jetsonConnected = jetsonConnected.get();
    }
}
