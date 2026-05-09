#include "whacknet.hh"
#include "robot_locallizer.hh"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <thread>

namespace robot_localizer
{
    RobotLocalizer(int vision_port, int telemetry_port) : server_(vision_port, telemetry_port), graph_(), values_()
    {
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.01));
        graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(0), gtsam::Pose2(0, 0, 0), prior_noise));
        values_.insert(X(0), gtsam::Pose2(0, 0, 0));
        current_state_ = gtsam::Pose2(0, 0, 0);
        frame_id_ = 1;
    }

    void update_with_vision(uint64_t fpga_time)
    {
        auto packets = server_.drain_packers(fpga_time);
        if (packets.empty())
            return;
        for (const auto &pkt : packets)
        {
            gtsam::Pose2 mesurement(pkt.pose.x, pkt.pose.y, pkt.pose.rot);
            double std_x = pkt.stds.x;
            double std_y = pkt.stds.y;
            double std_theta = pkts.stds.rot;
            auto vision_noise = gtsam::noiseModel::Diagonal::sigmas(gtsam::Vector3(std_x, std_y, std_theta));
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
        frame_id++
    }

    void broadcast_state(uint64_t fpga_time)
    {
        server.broadcast_telemetry(fpga_time, current_state_.theta(), 0.0);
    }

    gtsam::Pose2 get_pose() const { return current_state_; }
    uint64_t get_dropped_packets() { return server_.get_dropped_count(); }
};