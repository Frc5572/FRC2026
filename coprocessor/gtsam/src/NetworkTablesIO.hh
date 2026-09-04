#pragma once

#include <cstdint>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class NetworkTablesIO {
public:
    explicit NetworkTablesIO(nt::NetworkTableInstance inst);

    struct Inputs {
        double odomDx = 0.0;
        double odomDy = 0.0;
        double odomDtheta = 0.0;
        int64_t odomTimestampUs = 0;
        bool odomValid = false;

        double voDx = 0.0;
        double voDy = 0.0;
        double voDtheta = 0.0;
        double voConfidence = 0.0;
        int voInliers = 0;
        int64_t voTimestampUs = 0;
        bool voValid = false;

        double visionX = 0.0;
        double visionY = 0.0;
        double visionTheta = 0.0;
        double visionConfidence = 0.0;
        int64_t visionTimestampUs = 0;
        bool visionValid = false;

        bool reset = false;
        double resetX = 0.0;
        double resetY = 0.0;
        double resetTheta = 0.0;
        int64_t resetTimestampUs = 0;
    };

    Inputs read();
    void publish(double x, double y, double theta, int64_t timestampUs,
                 bool valid, bool visionAccepted, size_t graphSize);

private:
    std::shared_ptr<nt::NetworkTable> table_;

    nt::DoubleSubscriber odomDx_;
    nt::DoubleSubscriber odomDy_;
    nt::DoubleSubscriber odomDtheta_;
    nt::IntegerSubscriber odomTimestamp_;
    nt::BooleanSubscriber odomValid_;

    nt::DoubleSubscriber voDx_;
    nt::DoubleSubscriber voDy_;
    nt::DoubleSubscriber voDtheta_;
    nt::DoubleSubscriber voConfidence_;
    nt::IntegerSubscriber voInliers_;
    nt::IntegerSubscriber voTimestamp_;
    nt::BooleanSubscriber voValid_;

    nt::DoubleSubscriber visionX_;
    nt::DoubleSubscriber visionY_;
    nt::DoubleSubscriber visionTheta_;
    nt::DoubleSubscriber visionConfidence_;
    nt::IntegerSubscriber visionTimestamp_;
    nt::BooleanSubscriber visionValid_;

    nt::BooleanSubscriber reset_;
    nt::DoubleSubscriber resetX_;
    nt::DoubleSubscriber resetY_;
    nt::DoubleSubscriber resetTheta_;
    nt::IntegerSubscriber resetTimestamp_;

    nt::DoublePublisher outputX_;
    nt::DoublePublisher outputY_;
    nt::DoublePublisher outputTheta_;
    nt::IntegerPublisher outputTimestamp_;
    nt::BooleanPublisher outputValid_;
    nt::BooleanPublisher visionAccepted_;
    nt::IntegerPublisher graphSize_;
    nt::BooleanPublisher connected_;
};
