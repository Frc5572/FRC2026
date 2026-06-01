#include "localizer.hh"
#include "server.hh"
#include <gtsam/geometry/Pose2.h>

int main()
{
    Server nt;
    IsamLocalizer localizer;
    double lastOdomTimestamp = 0.0;
    double lastVisionTimestamp = 0.0;
    double poseTimestamp = 0.0;

    while (true)
    {
        gtsam::Pose2 pose = localizer.getLatestPose();

        double odomTimestamp = nt.readOdomTimestamp();
        if (odomTimestamp > lastOdomTimestamp)
        {
            gtsam::Pose2 odomDelta = nt.readOdomDelta();
            pose = localizer.addOdometry(odomDelta);
            lastOdomTimestamp = odomTimestamp;
            poseTimestamp = odomTimestamp;
        }

        int command = nt.pullCommand();

        double visionTimestamp = nt.readVisionTimestamp();
        if (nt.hasVision() && visionTimestamp > lastVisionTimestamp)
        {
            gtsam::Pose2 visionPose = nt.readVisionPose();
            auto [visionTranslationStdDev, visionRotStdDev] = nt.getVisionStdDev();

            if (visionTranslationStdDev > 0.0 && visionRotStdDev > 0.0)
            {
                pose = localizer.addVisionMeasurement(
                    visionPose, visionTranslationStdDev, visionRotStdDev);
                lastVisionTimestamp = visionTimestamp;
                poseTimestamp = visionTimestamp;
            }
        }

        nt.pubInited(localizer.isInited());

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

        nt.publishOptimizedPose(pose, poseTimestamp);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
