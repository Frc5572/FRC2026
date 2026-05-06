#include "whacknet.hh"

#include <arpa/inet.h>
#include <cstring>
#include <errno.h>
#include <time.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

namespace whacknet
{

    uint64_t WhacknetServer::get_monotonic_micros()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec * 1000000ULL +
               (uint64_t)ts.tv_nsec / 1000ULL;
    }

    uint64_t WhacknetServer::get_realtime_micros()
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        return (uint64_t)ts.tv_sec * 1000000ULL +
               (uint64_t)ts.tv_nsec / 1000ULL;
    }

    uint64_t WhacknetServer::extract_timestamp_from_cmsg(struct msghdr *msg)
    {
        for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg); cmsg != nullptr;
             cmsg = CMSG_NXTHDR(msg, cmsg))
        {
            if (cmsg->cmsg_level == SOL_SOCKET &&
                cmsg->cmsg_type == SCM_TIMESTAMP)
            {
                // macOS gives timeval, not timespec
                struct timeval *tv = (struct timeval *)CMSG_DATA(cmsg);

                return (uint64_t)tv->tv_sec * 1000000ULL +
                       (uint64_t)tv->tv_usec;
            }
        }

        return get_realtime_micros();
    }

    WhacknetServer::WhacknetServer(int recv_port, int broadcast_port)
        : broadcast_addr_{}
    {
        listen_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (listen_fd_ < 0)
        {
            perror("[Whacknet] Socket creation failed");
            return;
        }

        int rcvbuf = RECV_BUF_SIZE;
        setsockopt(listen_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        int reuse = 1;
        setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        // macOS supports SO_TIMESTAMP (timeval)
        int ts_on = 1;
        setsockopt(listen_fd_, SOL_SOCKET, SO_TIMESTAMP, &ts_on, sizeof(ts_on));

        struct sockaddr_in servaddr;
        std::memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        servaddr.sin_port = htons(recv_port);

        if (bind(listen_fd_, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
        {
            perror("[Whacknet] Bind failed");
            close(listen_fd_);
            listen_fd_ = -1;
            return;
        }

        printf("[Whacknet] Listening on port %d\n", recv_port);

        broadcast_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (broadcast_fd_ < 0)
        {
            perror("[Whacknet] Broadcast socket creation failed");
            return;
        }

        int broadcast = 1;
        setsockopt(broadcast_fd_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
        setsockopt(broadcast_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        std::memset(&broadcast_addr_, 0, sizeof(broadcast_addr_));
        broadcast_addr_.sin_family = AF_INET;
        broadcast_addr_.sin_port = htons(broadcast_port);
        broadcast_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);

        printf("[Whacknet] Broadcast socket ready on port %d\n", broadcast_port);

        start();
    }

    WhacknetServer::~WhacknetServer() { stop(); }

    void WhacknetServer::start()
    {
        if (listen_fd_ < 0)
        {
            fprintf(stderr, "[Whacknet] Cannot start\n");
            return;
        }

        should_run_.store(true);
        worker_thread_ = std::thread([this]()
                                     { receiver_worker(); });
    }

    void WhacknetServer::stop()
    {
        should_run_.store(false);

        if (worker_thread_.joinable())
            worker_thread_.join();

        if (listen_fd_ >= 0)
            close(listen_fd_);

        if (broadcast_fd_ >= 0)
            close(broadcast_fd_);
    }

    void WhacknetServer::broadcast_telemetry(uint64_t timestamp, double heading,
                                             double angular_velocity)
    {
        if (broadcast_fd_ < 0)
            return;

        GyroPacket pkt{timestamp, heading, angular_velocity};

        sendto(broadcast_fd_, &pkt, sizeof(pkt), 0,
               (struct sockaddr *)&broadcast_addr_,
               sizeof(broadcast_addr_));
    }

    std::vector<VisionMeasurement> WhacknetServer::drain_packets(uint64_t current_hal_time)
    {
        std::vector<VisionMeasurement> result;

        uint64_t now_monotonic = get_monotonic_micros();
        int64_t offset = (int64_t)current_hal_time - (int64_t)now_monotonic;

        int t = queue_.tail.load();
        int h = queue_.head.load();

        while (t != h && result.size() < MAX_QUEUE_SIZE)
        {
            VisionMeasurement m = queue_.data[t];

            m.timestamp_us = (uint64_t)((int64_t)m.timestamp_us + offset);

            result.push_back(m);
            t = (t + 1) & MASK;
        }

        queue_.tail.store(t);

        return result;
    }

    uint64_t WhacknetServer::get_dropped_count()
    {
        return queue_.dropped_packets.exchange(0);
    }

    void WhacknetServer::receiver_worker()
    {
        // macOS version (different API)
        pthread_setname_np("VisionUDPRecv");

        VisionMeasurement recv_bufs[RECV_BATCH];
        struct iovec iov;
        struct msghdr msg;

        union
        {
            char buf[CMSG_SPACE(sizeof(struct timeval))];
            struct timeval align;
        } ctrl_buf;

        while (should_run_.load())
        {
            for (int i = 0; i < RECV_BATCH; i++)
            {
                iov.iov_base = &recv_bufs[i];
                iov.iov_len = sizeof(VisionMeasurement);

                std::memset(&msg, 0, sizeof(msg));
                msg.msg_iov = &iov;
                msg.msg_iovlen = 1;
                msg.msg_control = ctrl_buf.buf;
                msg.msg_controllen = sizeof(ctrl_buf.buf);

                ssize_t len = recvmsg(listen_fd_, &msg, 0);
                if (len <= 0)
                    break;

                if (len != sizeof(VisionMeasurement))
                    continue;

                uint64_t ts = extract_timestamp_from_cmsg(&msg);

                int h = queue_.head.load();
                int t = queue_.tail.load();
                int next_h = (h + 1) & MASK;

                if (next_h == t)
                {
                    queue_.dropped_packets.fetch_add(1);
                    continue;
                }

                VisionMeasurement *pkt = &recv_bufs[i];
                pkt->timestamp_us = ts;

                queue_.data[h] = *pkt;
                queue_.head.store(next_h);
            }
        }
    }

} // namespace whacknet