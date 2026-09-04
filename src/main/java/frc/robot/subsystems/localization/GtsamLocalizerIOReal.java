package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GtsamLocalizerIOReal implements GtsamLocalizerIO {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("GtsamLocalizer");

    private final NetworkTable inputTable = table.getSubTable("Input");
    private final NetworkTable outputTable = table.getSubTable("Output");
    private final NetworkTable resetTable = table.getSubTable("Reset");

    private final DoubleSubscriber xSub = outputTable.getDoubleTopic("X").subscribe(0.0);
    private final DoubleSubscriber ySub = outputTable.getDoubleTopic("Y").subscribe(0.0);
    private final DoubleSubscriber thetaSub = outputTable.getDoubleTopic("Theta").subscribe(0.0);
    private final IntegerSubscriber timestampSub =
        outputTable.getIntegerTopic("TimestampUs").subscribe(0L);
    private final BooleanSubscriber validSub =
        outputTable.getBooleanTopic("Valid").subscribe(false);
    private final BooleanSubscriber visionAcceptedSub =
        outputTable.getBooleanTopic("VisionAccepted").subscribe(false);
    private final IntegerSubscriber graphSizeSub =
        outputTable.getIntegerTopic("GraphSize").subscribe(0L);

    private final DoublePublisher odomDxPub = inputTable.getDoubleTopic("DeltaX").publish();
    private final DoublePublisher odomDyPub = inputTable.getDoubleTopic("DeltaY").publish();
    private final DoublePublisher odomDthetaPub = inputTable.getDoubleTopic("DeltaTheta").publish();
    private final IntegerPublisher odomTimestampPub =
        inputTable.getIntegerTopic("TimestampUs").publish();
    private final edu.wpi.first.networktables.BooleanPublisher odomValidPub =
        inputTable.getBooleanTopic("Valid").publish();

    private final BooleanPublisher resetPub = resetTable.getBooleanTopic("Request").publish();
    private final DoublePublisher resetXPub = resetTable.getDoubleTopic("X").publish();
    private final DoublePublisher resetYPub = resetTable.getDoubleTopic("Y").publish();
    private final DoublePublisher resetThetaPub = resetTable.getDoubleTopic("Theta").publish();
    private final IntegerPublisher resetTimestampPub =
        resetTable.getIntegerTopic("TimestampUs").publish();

    @Override
    public void updateInputs(GtsamLocalizerIOInputs inputs) {
        inputs.xMeters = xSub.get();
        inputs.yMeters = ySub.get();
        inputs.thetaRadians = thetaSub.get();
        inputs.timestampUs = timestampSub.get();
        inputs.valid = validSub.get();
        inputs.visionAccepted = visionAcceptedSub.get();
        inputs.graphSize = graphSizeSub.get();
    }

    @Override
    public void setOdometryDelta(Pose2d delta, long timestampUs) {
        odomDxPub.set(delta.getX());
        odomDyPub.set(delta.getY());
        odomDthetaPub.set(delta.getRotation().getRadians());
        odomTimestampPub.set(timestampUs);
        odomValidPub.set(true);
    }

    @Override
    public void resetPose(Pose2d pose, long timestampUs) {
        resetXPub.set(pose.getX());
        resetYPub.set(pose.getY());
        resetThetaPub.set(pose.getRotation().getRadians());
        resetTimestampPub.set(timestampUs);
        resetPub.set(true);
    }
}
