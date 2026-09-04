#include "VisualOdometry.hpp"
#include "NetworkTablesPublisher.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <thread>

static double nowSeconds()
{
    using namespace std::chrono;

    return duration<double>(
               steady_clock::now().time_since_epoch())
        .count();
}

int main()
{
    std::cout
        << "=================================\n"
        << "       FRC Jetson SVO\n"
        << "=================================\n";

    /*
     * USB camera.
     */

    cv::VideoCapture camera(
        "/dev/video0",
        cv::CAP_V4L2);

    if (!camera.isOpened())
    {
        std::cerr
            << "ERROR: Could not open /dev/video0\n";

        return 1;
    }

    /*
     * Start with 640x480.
     */

    camera.set(
        cv::CAP_PROP_FRAME_WIDTH,
        640);

    camera.set(
        cv::CAP_PROP_FRAME_HEIGHT,
        480);

    camera.set(
        cv::CAP_PROP_FPS,
        60);

    std::cout
        << "Camera opened.\n";

    std::cout
        << "Resolution: "
        << camera.get(cv::CAP_PROP_FRAME_WIDTH)
        << "x"
        << camera.get(cv::CAP_PROP_FRAME_HEIGHT)
        << "\n";

    std::cout
        << "FPS: "
        << camera.get(cv::CAP_PROP_FPS)
        << "\n";

    /*
     * TEMPORARY camera intrinsics.
     *
     * Replace these with your calibrated values.
     */

    constexpr double FX = 520.0;
    constexpr double FY = 520.0;
    constexpr double CX = 320.0;
    constexpr double CY = 240.0;

    VisualOdometry vo(
        FX,
        FY,
        CX,
        CY);

    NetworkTablesPublisher ntPublisher;

    cv::Mat frame;

    uint64_t frameNumber = 0;

    while (true)
    {
        if (!camera.read(frame))
        {
            std::cerr
                << "Camera frame failure.\n";

            continue;
        }

        double timestamp =
            nowSeconds();

        bool success =
            vo.processFrame(
                frame,
                timestamp);

        const VOPose &pose =
            vo.getPose();

        /*
         * Publish to roboRIO.
         */

        ntPublisher.publish(
            pose,
            timestamp);

        /*
         * Debug display.
         */

        cv::putText(
            frame,
            "Features: " +
                std::to_string(
                    pose.featureCount),
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2);

        cv::putText(
            frame,
            "Inliers: " +
                std::to_string(
                    pose.inlierCount),
            cv::Point(20, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2);

        cv::putText(
            frame,
            "Quality: " +
                std::to_string(
                    pose.quality),
            cv::Point(20, 90),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2);

        cv::putText(
            frame,
            "Tracking: " +
                std::string(
                    pose.tracking
                        ? "YES"
                        : "NO"),
            cv::Point(20, 120),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2);

        if (frameNumber % 30 == 0)
        {
            std::cout
                << "X: "
                << pose.x
                << " Y: "
                << pose.y
                << " Z: "
                << pose.z
                << " Features: "
                << pose.featureCount
                << " Inliers: "
                << pose.inlierCount
                << "\n";
        }

        /*
         * Remove this section when running headless.
         */

        cv::imshow(
            "FRC SVO",
            frame);

        int key =
            cv::waitKey(1);

        if (key == 27 ||
            key == 'q')
        {
            break;
        }

        frameNumber++;
    }

    camera.release();

    cv::destroyAllWindows();

    return 0;
}