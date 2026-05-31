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
        int command = nt.pullCommand();

        // if (nt.hasVision())
        // {
        gtsam::Pose2 visionPose = nt.readVisionPose();
        auto [visionTranslationStdDev, visionRotStdDev] = nt.getVisionStdDev();
        pose = localizer.addVisionMeasurement(visionPose, visionTranslationStdDev, visionRotStdDev);
        nt.pubInited(localizer.isInited());
        // }
        switch (command)
        {
        case 0:
            break;
        case 1:
            nt.respondCommand(1);
            localizer.resetPose(nt.getCommandPose());
            nt.respondCommand(0);
            break;
        case 2:
            nt.respondCommand(1);
            localizer.resetTranslation(nt.getCommandPose().x(), nt.getCommandPose().y());
            nt.respondCommand(0);
            break;
        default:
            nt.respondCommand(0);
            break;
        }

        double timestamp = nt.readVisionTimestamp();
        nt.publishOptimizedPose(pose, timestamp);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
