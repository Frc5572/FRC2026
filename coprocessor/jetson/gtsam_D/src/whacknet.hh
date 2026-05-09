#pragma once

#include <cstdint>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <optional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>

namespace whacknet
{
    /// Robot pose from vision system
    struct RobotPose
    {
        double x, y, rot; // meters and radians
    };

    /// Vision measurement uncertainty (standard deviations)
    struct VisionUncertainty
    {
        double x, y, rot;
    };

    /// Complete vision measurement from camera
    struct VisionMeasurement
    {
        RobotPose pose;         // Estimated robot position
        VisionUncertainty stds; // Position uncertainty
        uint64_t timestamp_us;  // Monotonic timestamp in microseconds
        uint8_t camera_id;      // Which camera provided this
        uint8_t num_tags;       // Number of AprilTags detected
    };

    /// Gyroscope/telemetry data
    struct GyroPacket
    {
        uint64_t fpga_timestamp;
        double heading;
        double angular_velocity;
    };

    class WhacknetServer
    {
    public:
        /// Initialize the server on specified ports
        /// @param recv_port Port to listen for vision data
        /// @param broadcast_port Port for outgoing telemetry
        WhacknetServer(int recv_port, int broadcast_port);

        ~WhacknetServer();

        /// Disable copy operations
        WhacknetServer(const WhacknetServer &) = delete;
        WhacknetServer &operator=(const WhacknetServer &) = delete;

        /// Start the receiver thread (called automatically in constructor)
        void start();

        /// Stop the receiver thread
        void stop();

        /// Broadcast robot telemetry to the network
        /// @param timestamp FPGA timestamp
        /// @param heading Robot heading in radians
        /// @param angular_velocity Robot angular velocity
        void broadcast_telemetry(uint64_t timestamp, double heading,
                                 double angular_velocity);

        /// Drain all pending vision packets from the queue
        /// @param current_hal_time Current FPGA time for synchronization
        /// @return Vector of all pending vision measurements
        std::vector<VisionMeasurement> drain_packets(uint64_t current_hal_time);

        /// Get count of dropped packets (clears the counter)
        uint64_t get_dropped_count();

    private:
        static constexpr int MAX_QUEUE_SIZE = 64;
        static constexpr int MASK = MAX_QUEUE_SIZE - 1;
        static constexpr int RECV_BUF_SIZE = 4194304;
        static constexpr int RECV_BATCH = 16;
        static constexpr int CACHE_LINE = 64;
        static constexpr int RT_PRIORITY = 50;

        // Lock-free ring buffer
        struct
        {
            VisionMeasurement data[MAX_QUEUE_SIZE];
            alignas(CACHE_LINE) std::atomic<int> head{0};
            alignas(CACHE_LINE) std::atomic<int> tail{0};
            std::atomic<uint64_t> dropped_packets{0};
        } queue_;

        int listen_fd_ = -1;
        int broadcast_fd_ = -1;

        struct sockaddr_in broadcast_addr_;
        std::thread worker_thread_;
        std::atomic<bool> should_run_{false};

        void receiver_worker();

        // Utility functions
        static uint64_t get_monotonic_micros();
        static uint64_t get_realtime_micros();
        static uint64_t extract_timestamp_from_cmsg(struct msghdr *msg);
    };

} // namespace whacknet