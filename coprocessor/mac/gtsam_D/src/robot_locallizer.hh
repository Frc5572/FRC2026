#pragma once

#include "whacknet.hh"

#include <cstdint>
#include <gtsam/geometry/Pose2.h>
#include <string>

namespace robot_localizer
{

    class RobotLocalizer
    {
    public:
        RobotLocalizer(const std::string &rio_ip, int rio_vision_port,
                       int telemetry_port);

        void update_with_vision(uint64_t fpga_time_us);
        void update_with_wheel_odometry(uint64_t fpga_time_us);
        void broadcast_state(uint64_t fpga_time_us);

        gtsam::Pose2 get_pose() const;
        uint64_t get_dropped_packets() const;

    private:
        whacknet::WhacknetClient whacknet_;
        gtsam::Pose2 current_state_{0.0, 0.0, 0.0};
        uint8_t camera_id_ = 0;
        uint8_t num_tags_ = 0;
    };

} // namespace robot_localizer
