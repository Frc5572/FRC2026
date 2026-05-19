#include "whacknet.hh"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace whacknet
{

    WhacknetClient::WhacknetClient(const std::string &rio_ip, int rio_vision_port,
                                   int telemetry_port)
    {
        vision_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (vision_fd_ < 0)
        {
            std::perror("[Whacknet] vision socket creation failed");
            return;
        }

        std::memset(&rio_addr_, 0, sizeof(rio_addr_));
        rio_addr_.sin_family = AF_INET;
        rio_addr_.sin_port = htons(rio_vision_port);

        if (inet_pton(AF_INET, rio_ip.c_str(), &rio_addr_.sin_addr) != 1)
        {
            std::fprintf(stderr, "[Whacknet] invalid roboRIO IP address: %s\n",
                         rio_ip.c_str());
            close(vision_fd_);
            vision_fd_ = -1;
            return;
        }

        telemetry_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (telemetry_fd_ < 0)
        {
            std::perror("[Whacknet] telemetry socket creation failed");
            close(vision_fd_);
            vision_fd_ = -1;
            return;
        }

        int reuse = 1;
        setsockopt(telemetry_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        int rcvbuf = RECV_BUF_SIZE;
        setsockopt(telemetry_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        setsockopt(telemetry_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr.sin_port = htons(telemetry_port);

        if (bind(telemetry_fd_, reinterpret_cast<sockaddr *>(&local_addr),
                 sizeof(local_addr)) < 0)
        {
            std::perror("[Whacknet] telemetry bind failed");
            close(telemetry_fd_);
            close(vision_fd_);
            telemetry_fd_ = -1;
            vision_fd_ = -1;
            return;
        }

        std::printf("[Whacknet] sending vision to %s:%d\n", rio_ip.c_str(),
                    rio_vision_port);
        std::printf("[Whacknet] listening for roboRIO telemetry on port %d\n",
                    telemetry_port);

        start();
    }

    WhacknetClient::~WhacknetClient() { stop(); }

    bool WhacknetClient::ok() const
    {
        return vision_fd_ >= 0 && telemetry_fd_ >= 0;
    }

    void WhacknetClient::start()
    {
        if (!ok() || should_run_.exchange(true))
        {
            return;
        }

        receiver_thread_ = std::thread([this]
                                       { receiver_worker(); });
    }

    void WhacknetClient::stop()
    {
        should_run_.store(false);

        if (telemetry_fd_ >= 0)
        {
            close(telemetry_fd_);
            telemetry_fd_ = -1;
        }

        if (receiver_thread_.joinable())
        {
            receiver_thread_.join();
        }

        if (vision_fd_ >= 0)
        {
            close(vision_fd_);
            vision_fd_ = -1;
        }
    }

    bool WhacknetClient::send_vision_measurement(
        const CameraMeasurement &measurement)
    {
        if (vision_fd_ < 0)
        {
            return false;
        }

        ssize_t sent = sendto(vision_fd_, &measurement, sizeof(measurement), 0,
                              reinterpret_cast<sockaddr *>(&rio_addr_),
                              sizeof(rio_addr_));

        if (sent != static_cast<ssize_t>(sizeof(measurement)))
        {
            std::perror("[Whacknet] failed to send vision measurement");
            return false;
        }

        return true;
    }

    std::optional<RobotTelemetry> WhacknetClient::latest_telemetry() const
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        return latest_telemetry_;
    }

    uint64_t WhacknetClient::received_telemetry_count() const
    {
        return received_telemetry_count_.load();
    }

    uint64_t WhacknetClient::bad_telemetry_count() const
    {
        return bad_telemetry_count_.load();
    }

    void WhacknetClient::receiver_worker()
    {
#ifdef __APPLE__
        pthread_setname_np("WhacknetTelemetry");
#else
        pthread_setname_np(pthread_self(), "WhacknetTelemetry");
#endif

        while (should_run_.load())
        {
            RobotTelemetry packet{};
            sockaddr_in sender{};
            socklen_t sender_len = sizeof(sender);

            ssize_t len = recvfrom(telemetry_fd_, &packet, sizeof(packet), 0,
                                   reinterpret_cast<sockaddr *>(&sender), &sender_len);

            if (len < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ||
                    !should_run_.load())
                {
                    continue;
                }

                std::perror("[Whacknet] telemetry receive failed");
                continue;
            }

            if (len != static_cast<ssize_t>(sizeof(RobotTelemetry)))
            {
                bad_telemetry_count_.fetch_add(1);
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                latest_telemetry_ = packet;
            }

            received_telemetry_count_.fetch_add(1);
        }
    }

} // namespace whacknet