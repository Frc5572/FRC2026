#include "robot_locallizer.hh"

#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    robot_localizer::RobotLocalizer localizer(5800, 5801);

    const int TARGET_HZ = 120;
    const auto FRAME_DURATION = std::chrono::duration<double>(1.0 / TARGET_HZ);
    const int NUM_ITERATIONS = 1200; // 10 seconds at 120 Hz

    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t frame_count = 0;
    uint64_t missed_deadlines = 0;

    for (int frame = 0; frame < NUM_ITERATIONS; frame++)
    {
        auto frame_start = std::chrono::high_resolution_clock::now();

        uint64_t fpga_time =
            std::chrono::duration_cast<std::chrono::microseconds>(
                frame_start.time_since_epoch())
                .count();

        localizer.update_with_vision(fpga_time);

        auto pose = localizer.get_pose();
        std::cout << "[Frame " << frame << "] Estimated pose: "
                  << "x=" << pose.x() << " y=" << pose.y()
                  << " theta=" << pose.theta() << "\n";

        localizer.broadcast_state(fpga_time);

        uint64_t dropped = localizer.get_dropped_packets();
        if (dropped > 0)
        {
            std::cerr << "[Warning] Dropped " << dropped << " vision packets\n";
        }

        auto frame_end = std::chrono::high_resolution_clock::now();
        auto frame_duration = frame_end - frame_start;
        auto sleep_time = FRAME_DURATION - frame_duration;

        if (sleep_time.count() > 0)
        {
            std::this_thread::sleep_for(sleep_time);
        }
        else
        {
            std::cerr << "[Warning] Frame " << frame << " exceeded 120 Hz deadline by "
                      << -sleep_time.count() * 1000 << " us\n";
            missed_deadlines++;
        }

        frame_count++;

        if ((frame + 1) % 120 == 0)
        {
            std::cout << "\n--- Processed " << (frame + 1) << "/" << NUM_ITERATIONS
                      << " frames (" << (frame + 1) / 120 << "s) ---\n\n";
        }
    }

    auto total_time = std::chrono::high_resolution_clock::now() - start_time;
    auto total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(total_time);
    auto total_us =
        std::chrono::duration_cast<std::chrono::microseconds>(total_time);

    std::cout << "\n=== Simulation Complete ===\n";
    std::cout << "Total time: " << total_ms.count() << " ms ("
              << total_ms.count() / 1000.0 << " s)\n";
    std::cout << "Frames processed: " << frame_count << "\n";
    std::cout << "Target frequency: " << TARGET_HZ << " Hz\n";
    std::cout << "Average frame time: " << (total_us.count() / (double)frame_count)
              << " us\n";
    std::cout << "Missed deadlines: " << missed_deadlines << " / " << frame_count << "\n";

    if (missed_deadlines == 0)
    {
        std::cout << "✓ All frames met 120 Hz deadline!\n";
    }
    else
    {
        double miss_rate = (100.0 * missed_deadlines) / frame_count;
        std::cout << "✗ Miss rate: " << miss_rate << "%\n";
    }

    return 0;
}