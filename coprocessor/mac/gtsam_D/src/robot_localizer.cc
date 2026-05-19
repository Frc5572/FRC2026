#include "robot_localizer.hh"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace robot_localizer
{

    RobotLocalizer::RobotLocalizer(const std::string &rio_ip, int rio_vision_port,
                                   int telemetry_port)
        : whacknet_(rio_ip, rio_vision_port, telemetry_port) {}

    void RobotLocalizer::update_with_vision(uint64_t fpga_time)
    {
        auto packets = server_.drain_packets(fpga_time);
        if (packets.empty())
            return;
        for (const auto &pkt : packets)
        {
            gtsam::Pose2 measurement(pkt.pose.x, pkt.pose.y, pkt.pose.rot);
            double std_x = pkt.stds.x;
            double std_y = pkt.stds.y;
            double std_theta = pkt.stds.rot;
            auto vision_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(std_x, std_y, std_theta));
            graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(frame_id_), measurement, vision_noise));
        }

        try
        {
            gtsam::LevenbergMarquardtOptimizer optimizer(graph_, values_);
            values_ = optimizer.optimize();
            current_state_ = values_.at<gtsam::Pose2>(X(frame_id_));
        }
        catch (const std::exception &e)
        {
            std::cerr << "[GTSAM] Optimization failed: " << e.what() << "\n";
        }
        frame_id_++;
    }

    void RobotLocalizer::update_with_wheel_odometry(uint64_t fpga_time)
    {
        }

    void RobotLocalizer::broadcast_state(uint64_t fpga_time_us)
    {
        whacknet::VisionMeasurement measurement{};
        measurement.x = current_state_.x();
        measurement.y = current_state_.y();
        measurement.z = 0.0;
        measurement.roll = 0.0;
        measurement.pitch = 0.0;
        measurement.yaw = current_state_.theta();
        measurement.std_x = 0.1;
        measurement.std_y = 0.1;
        measurement.std_rot = 0.05;
        measurement.timestamp_us = fpga_time_us;
        measurement.camera_id = camera_id_;
        measurement.num_tags = num_tags_;

        if (!whacknet_.send_vision_measurement(measurement))
        {
            std::cerr << "[RobotLocalizer] Failed to send vision measurement\n";
        }
    }

    gtsam::Pose2 RobotLocalizer::get_pose() const { return current_state_; }

    uint64_t RobotLocalizer::get_dropped_packets() const
    {
        return whacknet_.bad_telemetry_count();
    }

} // namespace robot_localizer
