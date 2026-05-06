#include "whacknet.hh"

#include <arpa/inet.h>
#include <cstring>
#include <errno.h>
#include <linux/net_tstamp.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

namespace whacknet
{

    // ============================================================================
    // Utility Functions
    // ============================================================================

    uint64_t WhacknetServer::get_monotonic_micros()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
               static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
    }

    uint64_t WhacknetServer::get_realtime_micros()
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
               static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
    }

    uint64_t WhacknetServer::extract_timestamp_from_cmsg(struct msghdr *msg)
    {
        for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg); cmsg != nullptr;
             cmsg = CMSG_NXTHDR(msg, cmsg))
        {
            if (cmsg->cmsg_level == SOL_SOCKET &&
                cmsg->cmsg_type == SCM_TIMESTAMPNS)
            {
                struct timespec *ts = static_cast<struct timespec *>(CMSG_DATA(cmsg));
                return static_cast<uint64_t>(ts->tv_sec) * 1000000ULL +
                       static_cast<uint64_t>(ts->tv_nsec) / 1000ULL;
            }
        }
        // Fallback to system time if kernel timestamp unavailable
        return get_realtime_micros();
    }

    // ============================================================================
    // Constructor & Destructor
    // ============================================================================

    WhacknetServer::WhacknetServer(int recv_port, int broadcast_port)
        : broadcast_addr_{}
    {
        // Create UDP receive socket
        listen_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (listen_fd_ < 0)
        {
            perror("[Whacknet] Socket creation failed");
            return;
        }

        // Configure socket options
        int rcvbuf = RECV_BUF_SIZE;
        if (setsockopt(listen_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf,
                       sizeof(rcvbuf)) < 0)
        {
            perror("[Whacknet] SO_RCVBUF failed");
        }

        int reuse = 1;
        if (setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse,
                       sizeof(reuse)) < 0)
        {
            perror("[Whacknet] SO_REUSEADDR failed");
        }

        // Enable kernel nanosecond timestamps
        int ts_on = 1;
        if (setsockopt(listen_fd_, SOL_SOCKET, SO_TIMESTAMPNS, &ts_on,
                       sizeof(ts_on)) < 0)
        {
            perror("[Whacknet] Warning: SO_TIMESTAMPNS failed");
        }

        // Bind to receive port
        struct sockaddr_in servaddr;
        std::memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        servaddr.sin_port = htons(recv_port);

        if (bind(listen_fd_, reinterpret_cast<struct sockaddr *>(&servaddr),
                 sizeof(servaddr)) == -1)
        {
            perror("[Whacknet] Bind failed");
            close(listen_fd_);
            listen_fd_ = -1;
            return;
        }

        printf("[Whacknet] Listening on port %d\n", recv_port);

        // Create broadcast socket
        broadcast_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (broadcast_fd_ < 0)
        {
            perror("[Whacknet] Broadcast socket creation failed");
            return;
        }

        int broadcast = 1;
        if (setsockopt(broadcast_fd_, SOL_SOCKET, SO_BROADCAST, &broadcast,
                       sizeof(broadcast)) < 0)
        {
            perror("[Whacknet] SO_BROADCAST failed");
            close(broadcast_fd_);
            broadcast_fd_ = -1;
            return;
        }

        if (setsockopt(broadcast_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse,
                       sizeof(reuse)) < 0)
        {
            perror("[Whacknet] SO_REUSEADDR failed");
        }

        // Configure broadcast address
        std::memset(&broadcast_addr_, 0, sizeof(broadcast_addr_));
        broadcast_addr_.sin_family = AF_INET;
        broadcast_addr_.sin_port = htons(broadcast_port);
        broadcast_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);

        printf("[Whacknet] Broadcast socket ready on port %d\n", broadcast_port);

        // Start receiver thread
        start();
    }

    WhacknetServer::~WhacknetServer() { stop(); }

    // ============================================================================
    // Public Methods
    // ============================================================================

    void WhacknetServer::start()
    {
        if (listen_fd_ < 0)
        {
            fprintf(stderr, "[Whacknet] Cannot start: socket not initialized\n");
            return;
        }

        should_run_.store(true, std::memory_order_release);
        worker_thread_ = std::thread([this]()
                                     { receiver_worker(); });
    }

    void WhacknetServer::stop()
    {
        should_run_.store(false, std::memory_order_release);

        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }

        if (listen_fd_ >= 0)
        {
            close(listen_fd_);
            listen_fd_ = -1;
        }

        if (broadcast_fd_ >= 0)
        {
            close(broadcast_fd_);
            broadcast_fd_ = -1;
        }
    }

    void WhacknetServer::broadcast_telemetry(uint64_t timestamp, double heading,
                                             double angular_velocity)
    {
        if (broadcast_fd_ < 0)
            return;

        GyroPacket pkt{timestamp, heading, angular_velocity};
        sendto(broadcast_fd_, &pkt, sizeof(GyroPacket), 0,
               reinterpret_cast<struct sockaddr *>(&broadcast_addr_),
               sizeof(broadcast_addr_));
    }

    std::vector<VisionMeasurement> WhacknetServer::drain_packets(
        uint64_t current_hal_time)
    {
        std::vector<VisionMeasurement> result;

        // Calculate offset to sync monotonic clock to FPGA time
        uint64_t now_monotonic = get_monotonic_micros();
        int64_t offset = static_cast<int64_t>(current_hal_time) -
                         static_cast<int64_t>(now_monotonic);

        int t = queue_.tail.load(std::memory_order_relaxed);
        int h = queue_.head.load(std::memory_order_acquire);

        // Check for dropped packets periodically
        static uint64_t last_check = 0;
        if (now_monotonic - last_check > 2000000)
        { // 2 seconds
            uint64_t drops =
                queue_.dropped_packets.exchange(0, std::memory_order_relaxed);
            if (drops > 0)
            {
                printf("[Whacknet] Warning: Dropped %lu packets due to full queue\n",
                       drops);
            }
            last_check = now_monotonic;
        }

        // Drain all available packets
        while (t != h && result.size() < MAX_QUEUE_SIZE)
        {
            VisionMeasurement m = queue_.data[t];

            // Convert from monotonic to FPGA time
            m.timestamp_us = static_cast<uint64_t>(
                static_cast<int64_t>(m.timestamp_us) + offset);

            result.push_back(m);
            t = (t + 1) & MASK;
        }

        // Update ring buffer tail
        queue_.tail.store(t, std::memory_order_release);

        return result;
    }

    uint64_t WhacknetServer::get_dropped_count()
    {
        return queue_.dropped_packets.exchange(0, std::memory_order_relaxed);
    }

    // ============================================================================
    // Worker Thread
    // ============================================================================

    void WhacknetServer::receiver_worker()
    {
        // Set thread name
        pthread_setname_np(pthread_self(), "VisionUDPRecv");

        // Attempt to set real-time priority
        struct sched_param sp;
        sp.sched_priority = RT_PRIORITY;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        {
            fprintf(stderr,
                    "[Whacknet] Warning: SCHED_FIFO failed (need RT permissions)\n");
        }

        // Prepare batch receive structures
        VisionMeasurement recv_bufs[RECV_BATCH];
        struct iovec iovecs[RECV_BATCH];
        struct mmsghdr msgs[RECV_BATCH];

        union
        {
            char buf[CMSG_SPACE(sizeof(struct timespec))];
            struct timespec align;
        } ctrl_bufs[RECV_BATCH];

        std::memset(msgs, 0, sizeof(msgs));
        for (int i = 0; i < RECV_BATCH; i++)
        {
            iovecs[i].iov_base = &recv_bufs[i];
            iovecs[i].iov_len = sizeof(VisionMeasurement);

            msgs[i].msg_hdr.msg_iov = &iovecs[i];
            msgs[i].msg_hdr.msg_iovlen = 1;
            msgs[i].msg_hdr.msg_control = ctrl_bufs[i].buf;
            msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i].buf);
        }

        while (should_run_.load(std::memory_order_acquire))
        {
            // Reset control buffer sizes
            for (int i = 0; i < RECV_BATCH; i++)
            {
                msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i].buf);
            }

            // Receive batch of packets
            int n = recvmmsg(listen_fd_, msgs, RECV_BATCH, MSG_WAITFORONE, nullptr);
            if (n <= 0)
                continue;

            // Sample clock offset for this batch
            uint64_t m_now = get_monotonic_micros();
            uint64_t r_now = get_realtime_micros();
            int64_t r_to_m_offset = static_cast<int64_t>(m_now) - static_cast<int64_t>(r_now);

            int h = queue_.head.load(std::memory_order_relaxed);
            int t = queue_.tail.load(std::memory_order_acquire);

            // Process each received packet
            for (int i = 0; i < n; i++)
            {
                // Validate packet size
                if (msgs[i].msg_len != sizeof(VisionMeasurement))
                {
                    continue;
                }

                // Extract kernel timestamp
                uint64_t adapter_us_raw = extract_timestamp_from_cmsg(&msgs[i].msg_hdr);

                // Convert to monotonic time
                uint64_t adapter_us_monotonic =
                    static_cast<uint64_t>(static_cast<int64_t>(adapter_us_raw) +
                                          r_to_m_offset);

                VisionMeasurement *pkt = &recv_bufs[i];
                uint64_t abs_ts_monotonic = adapter_us_monotonic - pkt->timestamp_us;

                int next_h = (h + 1) & MASK;

                // Check if queue is full
                if (next_h == t)
                {
                    queue_.dropped_packets.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }

                // Write packet to queue
                pkt->timestamp_us = abs_ts_monotonic;
                queue_.data[h] = *pkt;

                queue_.head.store(next_h, std::memory_order_release);
                h = next_h;
            }
        }
    }

} // namespace whacknet