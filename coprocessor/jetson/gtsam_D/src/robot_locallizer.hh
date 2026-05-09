#include <chrono>
#include <iostream>
#include <thread>
#include <gtsam/geometry/Pose2.h>

namespace robot_localizer
{
    class RobotLocalizer
    {
    public:
        RobotLocalizer(int vision_port, int telemetry_port);

        void update_with_vision(uint64_t fpga_time);

        void broadcast_state(uint64_t fpga_time);

        gtsam::Pose2 getPose();

        uint64_t get_dropped_packets();

    private:
        whacknet::WhacknetServer server_;
        gtsam::NonlinearFactorGraph graph_;
        gtsam::Values values_;
        gtsam::Pose2 current_state_;
        size_t frame_id_;
    }
}