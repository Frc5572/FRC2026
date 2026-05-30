package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class GtsamServer {
    private DoublePublisher odomDxPub;
    private DoublePublisher odomDyPub;
    private DoublePublisher odomDthetaPub;
    private DoublePublisher odomTimestampPub;

    private BooleanPublisher visionValidPub;
    private DoublePublisher visionXPub;
    private DoublePublisher visionYPub;
    private DoublePublisher visionThetaPub;
    private DoublePublisher visionTimestampPub;
    private DoublePublisher visiontranslationStdDev;
    private DoublePublisher visionrotationStdDev;

    private DoubleSubscriber poseXSub;
    private DoubleSubscriber poseYSub;
    private DoubleSubscriber poseThetaSub;
    private DoubleSubscriber poseTimestampSub;

    private NetworkTableInstance inst;
    private final NetworkTable odomTable;
    private final NetworkTable visionTable;
    private final NetworkTable outputTable;

    public GtsamServer() {
        this.inst = NetworkTableInstance.getDefault();

        this.odomTable = inst.getTable("robot/odometry");

        this.odomDxPub = odomTable.getDoubleTopic("dx").publish();
        this.odomDyPub = odomTable.getDoubleTopic("dy").publish();
        this.odomDthetaPub = odomTable.getDoubleTopic("dtheta").publish();
        this.odomTimestampPub = odomTable.getDoubleTopic("timestamp").publish();

        this.visionTable = inst.getTable("vision/localizer");

        this.visionValidPub = visionTable.getBooleanTopic("valid").publish();
        this.visionXPub = visionTable.getDoubleTopic("x").publish();
        this.visionYPub = visionTable.getDoubleTopic("y").publish();
        this.visionThetaPub = visionTable.getDoubleTopic("theta").publish();
        this.visionTimestampPub = visionTable.getDoubleTopic("timestamp").publish();

        this.visiontranslationStdDev = visionTable.getDoubleTopic("XYStdDev").publish();
        this.visionrotationStdDev = visionTable.getDoubleTopic("rotStdDev").publish();

        this.outputTable = inst.getTable("localizer");

        this.poseXSub = outputTable.getDoubleTopic("x").subscribe(0.0);
        this.poseYSub = outputTable.getDoubleTopic("y").subscribe(0.0);
        this.poseThetaSub = outputTable.getDoubleTopic("theta").subscribe(0.0);
        this.poseTimestampSub = outputTable.getDoubleTopic("timestamp").subscribe(0.0);
    }

    public void updateOdom(Supplier<Pose2d> pose) {
        odomDxPub.set(pose.get().getX());
        odomDyPub.set(pose.get().getX());
        odomDthetaPub.set(pose.get().getRotation().getRadians());
        odomTimestampPub.set(Timer.getFPGATimestamp());
    }

    public void updateVision(Supplier<Pose2d> pose, DoubleSupplier timestamp,
        DoubleSupplier translationStdDev, DoubleSupplier rotationStdDev) {
        visionXPub.set(pose.get().getX());
        visionYPub.set(pose.get().getY());
        visionThetaPub.set(pose.get().getRotation().getRadians());
        visionTimestampPub.set(timestamp.getAsDouble());
        visiontranslationStdDev.set(translationStdDev.getAsDouble());
        visionrotationStdDev.set(rotationStdDev.getAsDouble());
    }

    public Pose2d getGtsamOptimization() {
        return new Pose2d(poseXSub.get(), poseYSub.get(),
            new Rotation2d(Radians.of(poseThetaSub.get())));
    }

    public void resetPose(Pose2d pose) {}

    public void resetTranslation(Translation2d translation) {}
}
