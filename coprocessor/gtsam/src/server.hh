#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>

#include <gtsam/geometry/Pose2.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

class Server
{
public:
    Server()
    {
        auto inst = nt::NetworkTableInstance::GetDefault();
        inst.StartClient4("gtsam-localizer");
        inst.SetServer("10.55.72.2");

        auto odomTable = inst.GetTable("robot/odometry");
        auto visionTable = inst.GetTable("vision/localizer");
        auto outputTable = inst.GetTable("localizer");
        auto commandTable = inst.GetTable("command");

        odomDxSub_ = odomTable->GetDoubleTopic("dx").Subscribe(0.0);
        odomDySub_ = odomTable->GetDoubleTopic("dy").Subscribe(0.0);
        odomDthetaSub_ = odomTable->GetDoubleTopic("dtheta").Subscribe(0.0);
        odomTimestampSub_ = odomTable->GetDoubleTopic("timestamp").Subscribe(0.0);

        visionValidSub_ = visionTable->GetBooleanTopic("valid").Subscribe(false);
        visionXSub_ = visionTable->GetDoubleTopic("x").Subscribe(0.0);
        visionYSub_ = visionTable->GetDoubleTopic("y").Subscribe(0.0);
        visionThetaSub_ = visionTable->GetDoubleTopic("theta").Subscribe(0.0);
        visionTimestampSub_ = visionTable->GetDoubleTopic("timestamp").Subscribe(0.0);

        poseXPub_ = outputTable->GetDoubleTopic("x").Publish();
        poseYPub_ = outputTable->GetDoubleTopic("y").Publish();
        poseThetaPub_ = outputTable->GetDoubleTopic("theta").Publish();
        poseTimestampPub_ = outputTable->GetDoubleTopic("timestamp").Publish();

        command_ = commandTable->GetStringTopic("command").Subscribe("");
        commandResponse_ = commandTable->GetStringTopic("commandResponse").Publish();
        commandXPub_ = commandTable->GetDoubleTopic("x").Subscribe(0.0);
        commandYPub_ = commandTable->GetDoubleTopic("y").Subscribe(0.0);
        commandThetaPub_ = commandTable->GetDoubleTopic("theta").Subscribe(0.0);
        commandTimestampPub_ = commandTable->GetDoubleTopic("timestamp").Subscribe(0.0);
    }

    gtsam::Pose2 readOdomDelta()
    {
        return gtsam::Pose2(
            odomDxSub_.Get(),
            odomDySub_.Get(),
            odomDthetaSub_.Get());
    }

    bool hasVision()
    {
        return visionValidSub_.Get();
    }

    gtsam::Pose2 readVisionPose()
    {
        return gtsam::Pose2(
            visionXSub_.Get(),
            visionYSub_.Get(),
            visionThetaSub_.Get());
    }

    double readVisionTimestamp()
    {
        return visionTimestampSub_.Get();
    }

    void publishOptimizedPose(const gtsam::Pose2 &pose, double timestamp)
    {
        poseXPub_.Set(pose.x());
        poseYPub_.Set(pose.y());
        poseThetaPub_.Set(pose.theta());
        poseTimestampPub_.Set(timestamp);
    }

    string pullCommand()
    {
        return command_.get();
    }

    void respondCommand(string &response)
    {
        commandResponse_.Set(response);
    }

    gtsam::Pose2 readCommandPose()
    {
        return gtsam::Pose2(
            commandXSub_.get(),
            commandYSub_.get(),
            commandThetaSub_.get(), )
    }

private:
    nt::DoubleSubscriber odomDxSub_;
    nt::DoubleSubscriber odomDySub_;
    nt::DoubleSubscriber odomDthetaSub_;
    nt::DoubleSubscriber odomTimestampSub_;

    nt::BooleanSubscriber visionValidSub_;
    nt::DoubleSubscriber visionXSub_;
    nt::DoubleSubscriber visionYSub_;
    nt::DoubleSubscriber visionThetaSub_;
    nt::DoubleSubscriber visionTimestampSub_;

    nt::DoublePublisher poseXPub_;
    nt::DoublePublisher poseYPub_;
    nt::DoublePublisher poseThetaPub_;
    nt::DoublePublisher poseTimestampPub_;

    nt::StringSubscriber command_;
    nt::StringPublisher commandResponse_;
    nt::DoublePublisher commandXSub_;
    nt::DoublePublisher commandYSub_;
    nt::DoublePublisher commandThetaSub_;
    nt::DoublePublisher commandTimestampSub_;
};