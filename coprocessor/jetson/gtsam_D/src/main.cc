#include "whacknet.hh"

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

using gtsam::symbol_shorthand::X;

// ============================================================================
// Robot Localization with Vision + GTSAM
// ============================================================================

class RobotLocalizer
{
public:
    RobotLocalizer(int vision_port, int telemetry_port)
        : server_(vision_port, telemetry_port), graph_(), values_()
    {
        // Initialize prior at origin
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector3(0.1, 0.1, 0.01)); // x, y, theta

        graph_.addPriorFactor(X(0), gtsam::Pose2(0, 0, 0), prior_noise);
        values_.insert(X(0), gtsam::Pose2(0, 0, 0));

        current_state_ = gtsam::Pose2(0, 0, 0);
        frame_id_ = 1;
    }

    /// Process vision measurements and update estimate
    void update_with_vision(uint64_t fpga_time)
    {
        auto packets = server_.drain_packets(fpga_time);

        if (packets.empty())
            return;

        // Fuse all vision packets received in this frame
        for (const auto &pkt : packets)
        {
            // Create vision measurement from camera data
            gtsam::Pose2 measurement(pkt.pose.x, pkt.pose.y, pkt.pose.rot);

            // Vision uncertainty (from camera)
            double std_x = pkt.stds.x;
            double std_y = pkt.stds.y;
            double std_theta = pkt.stds.rot;

            auto vision_noise = gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector3(std_x, std_y, std_theta));

            // Add pose measurement
            graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose2d>>(X(0), gtsam::Pose2(0, 0, 0), vision_noise);

            std::cout << "[Vision] Camera " << (int)pkt.camera_id << ": "
                      << "x=" << pkt.pose.x << " y=" << pkt.pose.y
                      << " rot=" << pkt.pose.rot << " tags=" << (int)pkt.num_tags
                      << "\n";
        }

        // Optimize graph
        try
        {
            gtsam::LevenbergMarquardtOptimizer optimizer(graph_, values_);
            values_ = optimizer.optimize();

            current_state_ = values_.at<gtsam::Pose2>(X(frame_id_));

            std::cout << "[GTSAM] Optimized pose: "
                      << "x=" << current_state_.x() << " y=" << current_state_.y()
                      << " theta=" << current_state_.theta() << "\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "[GTSAM] Optimization failed: " << e.what() << "\n";
        }

        frame_id_++;
    }

    /// Broadcast current state estimate
    void broadcast_state(uint64_t fpga_time)
    {
        server_.broadcast_telemetry(fpga_time, current_state_.theta(), 0.0);
    }

    /// Get current pose estimate
    gtsam::Pose2 get_pose() const { return current_state_; }

    /// Get dropped packet count for diagnostics
    uint64_t get_dropped_packets() { return server_.get_dropped_count(); }

private:
    whacknet::WhacknetServer server_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values values_;
    gtsam::Pose2 current_state_;
    size_t frame_id_;
};

// ============================================================================
// Main Robot Control Loop
// ============================================================================

int main()
{
    std::cout << "=== GTSAM-based Robot Localizer ===\n";

    // Create localizer (listening on 5800, broadcasting on 5801)
    RobotLocalizer localizer(5800, 5801);

    // Simulate robot operation
    const int NUM_ITERATIONS = 100;
    const std::chrono::milliseconds FRAME_TIME(50); // 20 Hz

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < NUM_ITERATIONS; frame++)
    {
        auto frame_start = std::chrono::high_resolution_clock::now();
        uint64_t fpga_time =
            std::chrono::duration_cast<std::chrono::microseconds>(
                frame_start.time_since_epoch())
                .count();

        // Process incoming vision data and update state
        localizer.update_with_vision(fpga_time);

        // Get current estimate
        auto pose = localizer.get_pose();
        std::cout << "[Frame " << frame << "] Estimated pose: "
                  << "x=" << pose.x() << " y=" << pose.y()
                  << " theta=" << pose.theta() << "\n";

        // Broadcast telemetry
        localizer.broadcast_state(fpga_time);

        // Check for dropped packets
        uint64_t dropped = localizer.get_dropped_packets();
        if (dropped > 0)
        {
            std::cerr << "[Warning] Dropped " << dropped << " vision packets\n";
        }

        // Wait until next frame time
        auto frame_end = std::chrono::high_resolution_clock::now();
        auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            frame_end - frame_start);

        if (frame_duration < FRAME_TIME)
        {
            std::this_thread::sleep_for(FRAME_TIME - frame_duration);
        }
        else
        {
            std::cerr << "[Warning] Frame took " << frame_duration.count()
                      << "ms (target: " << FRAME_TIME.count() << "ms)\n";
        }

        // Print progress every 10 frames
        if ((frame + 1) % 10 == 0)
        {
            std::cout << "\n--- Processed " << (frame + 1) << "/" << NUM_ITERATIONS
                      << " frames ---\n\n";
        }
    }

    auto total_time = std::chrono::high_resolution_clock::now() - start_time;
    auto total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(total_time);

    std::cout << "\n=== Simulation Complete ===\n";
    std::cout << "Total time: " << total_ms.count() << " ms\n";
    std::cout << "Average frame time: " << (total_ms.count() / NUM_ITERATIONS)
              << " ms\n";

    return 0;
}