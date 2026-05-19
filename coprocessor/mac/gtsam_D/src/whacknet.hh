#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <string>
#include <thread>

namespace whacknet
{

    constexpr int MAX_CAMERAS = 4;

    struct CameraMeasurement
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        double std_x;
        double std_y;
        double std_rot;

        uint64_t timestamp_us;
        uint8_t valid;
        uint8_t padding[7]{};
    };

    static_assert(sizeof(CameraMeasurement) == 88);

    struct VisionPacket
    {
        CameraMeasurement cameras[MAX_CAMERAS];
    };

    static_assert(sizeof(VisionPacket) == MAX_CAMERAS * 88);

    struct OdomMeasurement
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;

        double std_x;
        double std_y;
        double std_rot;

        uint64_t timestamp_us;

        uint8_t padding[16]{};
    };

    static_assert(sizeof(OdomMeasurement) == 96,
                  "OdomMeasurement must match Java STRUCT_SIZE");

    // Must match Java broadcastTelemetry():
    // long timestampUs + 6 doubles = 56 bytes sent by DatagramChannel.send().
    struct RobotTelemetry
    {
        uint64_t timestamp_us;
        double roll;
        double pitch;
        double yaw;
        double roll_vel;
        double pitch_vel;
        double yaw_vel;
    };

    static_assert(sizeof(RobotTelemetry) == 56,
                  "RobotTelemetry must match Java telemetry payload size");

    class WhacknetClient
    {
    public:
        // rio_ip is normally "10.TE.AM.2". For team 5572, that is "10.55.72.2".
        // rio_vision_port must match Java: Whacknet.getInstance().startServer(port).
        // telemetry_port must match Java: registerTelemetryService(port, ...).
        WhacknetClient(const std::string &rio_ip, int rio_vision_port,
                       int telemetry_port);
        ~WhacknetClient();

        WhacknetClient(const WhacknetClient &) = delete;
        WhacknetClient &operator=(const WhacknetClient &) = delete;

        bool ok() const;

        void start();
        void stop();

        bool send_vision_measurement(const CameraMeasurement &measurement);

        std::optional<RobotTelemetry> latest_telemetry() const;
        uint64_t received_telemetry_count() const;
        uint64_t bad_telemetry_count() const;

    private:
        static constexpr int RECV_BUF_SIZE = 4 * 1024 * 1024;

        int telemetry_fd_ = -1;
        int vision_fd_ = -1;

        sockaddr_in rio_addr_{};
        std::thread receiver_thread_;
        std::atomic<bool> should_run_{false};

        mutable std::mutex telemetry_mutex_;
        std::optional<RobotTelemetry> latest_telemetry_;

        std::atomic<uint64_t> received_telemetry_count_{0};
        std::atomic<uint64_t> bad_telemetry_count_{0};

        void receiver_worker();
    };

} // namespace whacknet