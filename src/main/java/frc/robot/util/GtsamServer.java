package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class GtsamServer {
    private static final double odomPublishPeriodSecs = 0.10;
    private static final double minOdomTranslationMeters = 0.05;
    private static final double minOdomRotationRadians = 0.035;
    private static final double poseSmoothingAlpha = 0.15;

    private DoublePublisher odomDxPub;
    private DoublePublisher odomDyPub;
    private DoublePublisher odomDthetaPub;
    private DoublePublisher odomTimestampPub;

    private BooleanPublisher visionValidPub;
    private BooleanSupplier visionInited;
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

    private IntegerPublisher command;
    private IntegerSubscriber commandResp;
    private DoublePublisher commandXPub;
    private DoublePublisher commandYPub;
    private DoublePublisher commandThetaPub;
    private DoublePublisher commandTimestampPub;

    private NetworkTableInstance inst;
    private final NetworkTable odomTable;
    private final NetworkTable visionTable;
    private final NetworkTable outputTable;
    private final NetworkTable commandTable;
    private Pose2d lastOdomPose = Pose2d.kZero;
    private boolean hasLastOdomPose = false;
    private double lastOdomPublishTime = 0.0;
    private Pose2d latestOdomPose = Pose2d.kZero;
    private Transform2d smoothedCorrection = Transform2d.kZero;
    private double lastPoseTimestamp = 0.0;
    private boolean hasLatestOdomPose = false;

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
        this.visionInited = visionTable.getBooleanTopic("inited").subscribe(false);
        this.visiontranslationStdDev = visionTable.getDoubleTopic("translationStdDev").publish();
        this.visionrotationStdDev = visionTable.getDoubleTopic("rotStdDev").publish();

        this.outputTable = inst.getTable("localizer");

        this.poseXSub = outputTable.getDoubleTopic("x").subscribe(0.0);
        this.poseYSub = outputTable.getDoubleTopic("y").subscribe(0.0);
        this.poseThetaSub = outputTable.getDoubleTopic("theta").subscribe(0.0);
        this.poseTimestampSub = outputTable.getDoubleTopic("timestamp").subscribe(0.0);

        this.commandTable = inst.getTable("command");
        this.command = commandTable.getIntegerTopic("command").publish();
        this.commandResp = commandTable.getIntegerTopic("commandResponse").subscribe(0);
        this.commandXPub = commandTable.getDoubleTopic("x").publish();
        this.commandYPub = commandTable.getDoubleTopic("y").publish();
        this.commandThetaPub = commandTable.getDoubleTopic("theta").publish();
        this.commandTimestampPub = commandTable.getDoubleTopic("timestamp").publish();

    }

    public void updateOdom(Supplier<Pose2d> pose) {
        Pose2d currentPose = pose.get();
        double now = Timer.getFPGATimestamp();
        latestOdomPose = currentPose;
        hasLatestOdomPose = true;

        if (!hasLastOdomPose) {
            lastOdomPose = currentPose;
            hasLastOdomPose = true;
            odomDxPub.set(0.0);
            odomDyPub.set(0.0);
            odomDthetaPub.set(0.0);
            odomTimestampPub.set(now);
            lastOdomPublishTime = now;
            return;
        }

        Transform2d odomDelta = currentPose.minus(lastOdomPose);
        boolean movedEnough =
            Math.hypot(odomDelta.getX(), odomDelta.getY()) >= minOdomTranslationMeters
                || Math.abs(odomDelta.getRotation().getRadians()) >= minOdomRotationRadians;
        if (!movedEnough || now - lastOdomPublishTime < odomPublishPeriodSecs) {
            return;
        }

        lastOdomPose = currentPose;
        lastOdomPublishTime = now;

        odomDxPub.set(odomDelta.getX());
        odomDyPub.set(odomDelta.getY());
        odomDthetaPub.set(odomDelta.getRotation().getRadians());
        odomTimestampPub.set(now);
    }

    public void updateVision(Pose2d pose, double timestamp, double translationStdDev,
        double rotationStdDev) {
        visionXPub.set(pose.getX());
        visionYPub.set(pose.getY());
        visionThetaPub.set(pose.getRotation().getRadians());
        visionTimestampPub.set(timestamp);
        visiontranslationStdDev.set(translationStdDev);
        visionrotationStdDev.set(rotationStdDev);
        visionValidPub.set(true);
    }

    public Pose2d getGtsamOptimization() {
        Pose2d rawGtsamPose = new Pose2d(poseXSub.get(), poseYSub.get(),
            new Rotation2d(Radians.of(poseThetaSub.get())));
        if (!hasLatestOdomPose) {
            return rawGtsamPose;
        }

        double timestamp = poseTimestampSub.get();
        if (timestamp > 0.0 && timestamp != lastPoseTimestamp) {
            Transform2d targetCorrection = rawGtsamPose.minus(latestOdomPose);
            smoothedCorrection = new Transform2d(
                smoothedCorrection.getX()
                    + (targetCorrection.getX() - smoothedCorrection.getX()) * poseSmoothingAlpha,
                smoothedCorrection.getY()
                    + (targetCorrection.getY() - smoothedCorrection.getY()) * poseSmoothingAlpha,
                new Rotation2d(smoothedCorrection.getRotation().getRadians()
                    + targetCorrection.getRotation().minus(smoothedCorrection.getRotation())
                        .getRadians() * poseSmoothingAlpha));
            lastPoseTimestamp = timestamp;
        }

        return latestOdomPose.plus(smoothedCorrection);
    }

    public void resetPose(Pose2d pose) {
        command.set(1);
        commandXPub.set(pose.getX());
        commandYPub.set(pose.getY());
        commandThetaPub.set(pose.getRotation().getRadians());
        commandTimestampPub.set(Timer.getFPGATimestamp());
        while (!(0 == commandResp.get())) {
        }
        lastOdomPose = pose;
        hasLastOdomPose = true;
        latestOdomPose = pose;
        hasLatestOdomPose = true;
        smoothedCorrection = Transform2d.kZero;
        command.set(0);
    }

    public void resetTranslation(Translation2d translation) {
        command.set(2);
        commandXPub.set(translation.getX());
        commandYPub.set(translation.getY());
        commandTimestampPub.set(Timer.getFPGATimestamp());
        while (!(0 == commandResp.get())) {
        }
        command.set(0);
    }

    public boolean isInited() {
        return visionInited.getAsBoolean();
    }

}
