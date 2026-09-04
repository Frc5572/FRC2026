#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <networktables/NetworkTableInstance.h>

#include "GtsamLocalizer.h"
#include "NetworkTablesIO.h"

namespace {
constexpr int kDefaultTeam = 0;
constexpr const char* kClientName = "gtsam-localizer";

// NT4 timestamps are microseconds. GTSAM only needs relative ordering here;
// timestamps are retained so stale measurements can be rejected.
constexpr int64_t kMaxMeasurementAgeUs = 250000;
}

int main(int argc, char** argv) {
    int team = kDefaultTeam;
    if (argc > 1) {
        team = std::stoi(argv[1]);
    }

    auto inst = nt::NetworkTableInstance::GetDefault();
    inst.StartClient4(kClientName);
    if (team > 0) {
        inst.SetServerTeam(team);
    } else {
        inst.SetServer("roborio");
    }

    NetworkTablesIO ntio(inst);
    GtsamLocalizer localizer;

    std::cout << "GTSAM FRC localizer started" << std::endl;
    std::cout << "NetworkTables server: team " << team << std::endl;

    int64_t lastOdomTimestamp = 0;
    int64_t lastVoTimestamp = 0;
    int64_t lastVisionTimestamp = 0;
    int64_t lastResetTimestamp = 0;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const int64_t localNowUs =
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()).count();

        auto in = ntio.read();

        if (in.reset && in.resetTimestampUs != lastResetTimestamp) {
            localizer.reset(
                gtsam::Pose2(in.resetX, in.resetY, in.resetTheta),
                in.resetTimestampUs);
            lastResetTimestamp = in.resetTimestampUs;
        }

        if (in.odomValid && in.odomTimestampUs != lastOdomTimestamp) {
            localizer.addOdometry(
                gtsam::Pose2(in.odomDx, in.odomDy, in.odomDtheta),
                in.odomTimestampUs);
            lastOdomTimestamp = in.odomTimestampUs;
        }

        // Visual odometry is relative motion. It is useful for improving local
        // motion estimates, but it cannot establish field position by itself.
        if (in.voValid && in.voTimestampUs != lastVoTimestamp &&
            in.voTimestampUs > 0) {
            localizer.addVisualOdometry(
                gtsam::Pose2(in.voDx, in.voDy, in.voDtheta),
                in.voConfidence,
                in.voInliers,
                in.voTimestampUs);
            lastVoTimestamp = in.voTimestampUs;
        }

        // Absolute vision should be field-referenced (AprilTag, field map, etc.).
        if (in.visionValid && in.visionTimestampUs != lastVisionTimestamp &&
            in.visionTimestampUs > 0) {
            localizer.addAbsoluteVision(
                gtsam::Pose2(in.visionX, in.visionY, in.visionTheta),
                in.visionConfidence,
                in.visionTimestampUs);
            lastVisionTimestamp = in.visionTimestampUs;
        }

        const auto pose = localizer.pose();
        const bool valid = localizer.initialized();
        ntio.publish(
            pose.x(), pose.y(), pose.theta(),
            localizer.timestampUs(), valid,
            localizer.lastVisionAccepted(),
            localizer.graphSize());

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
