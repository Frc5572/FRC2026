#include "localizer.hh"
#include "server.hh"
#include <gtsam/geometry/Pose2.h>

int main()
{
    Server nt;
    IsamLocalizer localizer;

    while (true)
    {
        gtsam::Pose2 odomDelta = nt.readOdomDelta();
        gtsam::Pose2 pose = localizer.addOdometry(odomDelta);

        if (nt.hasVision())
        {
            gtsam::Pose2 visionPose = nt.readVisionPose();
            pose = localizer.addVisionMeasurement(visionPose);
        }

        double timestamp = nt.readVisionTimestamp();
        nt.publishOptimizedPose(pose, timestamp);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
