#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif

#include <ntcore.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

struct TransformResult
{
    std::vector<cv::DMatch> rigidInliers;
    std::vector<cv::DMatch> homographyInliers;
    cv::Mat rigidTransform;
    bool success = false;
};

struct Frame
{
    cv::Mat gray;
    cv::Mat processed;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

static cv::Mat buildOverexposureMask(const cv::Mat &gray)
{
    cv::Mat mask;
    cv::threshold(gray, mask, 250, 255, cv::THRESH_BINARY_INV);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    return mask;
}

static cv::Mat contrastStretch(
    const cv::Mat &gray,
    const cv::Mat &mask)
{

    auto clahe =
        cv::createCLAHE(2.0, cv::Size(10, 10));

    cv::Mat stretched;
    clahe->apply(gray, stretched);

    if (!mask.empty())
    {
        cv::Mat result = gray.clone();
        stretched.copyTo(result, mask);
        return result;
    }

    return stretched;
}

static cv::Ptr<cv::Feature2D> createDetector()
{
#ifdef HAVE_OPENCV_XFEATURES2D
    std::cout << "Using Star/CenSurE detector\n";
    return cv::xfeatures2d::StarDetector::create();
#else
    std::cerr
        << "WARNING: StarDetector unavailable; using FAST.\n"
        << "Install/build OpenCV with opencv_contrib for Star/CenSurE.\n";

    return cv::FastFeatureDetector::create(
        20,
        true,
        cv::FastFeatureDetector::TYPE_9_16);
#endif
}

static cv::Ptr<cv::ORB> createDescriptor()
{
    return cv::ORB::create(
        800,
        1.2f,
        8,
        5,
        0,
        2,
        cv::ORB::HARRIS_SCORE,
        31,
        20);
}

static std::vector<cv::DMatch> matchDescriptors(
    const cv::Mat &a,
    const cv::Mat &b,
    double ratio = 0.8)
{

    if (a.empty() || b.empty())
        return {};

    cv::BFMatcher matcher(
        cv::NORM_HAMMING,
        false);

    std::vector<std::vector<cv::DMatch>> ab;
    std::vector<std::vector<cv::DMatch>> ba;

    matcher.knnMatch(a, b, ab, 2);
    matcher.knnMatch(b, a, ba, 2);

    std::vector<cv::DMatch> goodAB;

    for (const auto &pair : ab)
    {
        if (pair.size() < 2)
            continue;

        if (pair[0].distance <
            ratio * pair[1].distance)
        {
            goodAB.push_back(pair[0]);
        }
    }

    std::vector<std::pair<int, int>> goodBA;

    for (const auto &pair : ba)
    {
        if (pair.size() < 2)
            continue;

        if (pair[0].distance <
            ratio * pair[1].distance)
        {
            goodBA.emplace_back(
                pair[0].trainIdx,
                pair[0].queryIdx);
        }
    }

    std::vector<cv::DMatch> result;

    for (const auto &m : goodAB)
    {
        for (const auto &p : goodBA)
        {
            if (p.first == m.queryIdx &&
                p.second == m.trainIdx)
            {
                result.push_back(m);
                break;
            }
        }
    }

    return result;
}

static TransformResult fitTransform(
    const Frame &a,
    const Frame &b,
    const std::vector<cv::DMatch> &matches)
{

    TransformResult result;

    if (matches.size() < 4)
        return result;

    std::vector<cv::Point2f> ptsA;
    std::vector<cv::Point2f> ptsB;

    for (const auto &m : matches)
    {
        ptsA.push_back(
            a.keypoints[m.queryIdx].pt);

        ptsB.push_back(
            b.keypoints[m.trainIdx].pt);
    }

    cv::Mat mask;

    result.rigidTransform =
        cv::estimateAffinePartial2D(
            ptsA,
            ptsB,
            mask,
            cv::RANSAC,
            8.0,
            10000,
            0.999);

    if (result.rigidTransform.empty())
        return result;

    std::vector<cv::Point2f> rigidA;
    std::vector<cv::Point2f> rigidB;

    for (int i = 0; i < mask.rows; ++i)
    {
        if (mask.at<uchar>(i))
        {
            result.rigidInliers.push_back(matches[i]);
            rigidA.push_back(ptsA[i]);
            rigidB.push_back(ptsB[i]);
        }
    }

    if (rigidA.size() >= 4)
    {
        cv::Mat homographyMask;

        cv::Mat H =
            cv::findHomography(
                rigidA,
                rigidB,
                cv::RANSAC,
                3.0,
                homographyMask);

        if (!H.empty())
        {
            for (int i = 0;
                 i < homographyMask.rows;
                 ++i)
            {

                if (homographyMask.at<uchar>(i))
                    result.homographyInliers.push_back(
                        result.rigidInliers[i]);
            }
        }
    }

    result.success =
        result.homographyInliers.size() >= 25;

    return result;
}

static Frame processFrame(
    const cv::Mat &input,
    const cv::Ptr<cv::Feature2D> &detector,
    const cv::Ptr<cv::ORB> &descriptor,
    int roiRadius)
{

    Frame frame;

    frame.gray = input.clone();

    cv::Mat mask =
        buildOverexposureMask(frame.gray);

    frame.processed =
        contrastStretch(frame.gray, mask);

    int cx = frame.processed.cols / 2;
    int cy = frame.processed.rows / 2;

    int x0 = std::max(0, cx - roiRadius);
    int y0 = std::max(0, cy - roiRadius);
    int x1 = std::min(
        frame.processed.cols,
        cx + roiRadius);
    int y1 = std::min(
        frame.processed.rows,
        cy + roiRadius);

    cv::Rect roi(
        x0,
        y0,
        x1 - x0,
        y1 - y0);

    cv::Mat crop =
        frame.processed(roi);

    detector->detect(
        crop,
        frame.keypoints);

    descriptor->compute(
        crop,
        frame.keypoints,
        frame.descriptors);

    for (auto &kp : frame.keypoints)
    {
        kp.pt.x +=
            static_cast<float>(x0);

        kp.pt.y +=
            static_cast<float>(y0);
    }

    return frame;
}

static bool extractMotion(
    const cv::Mat &T,
    double metersPerPixel,
    double &dxMeters,
    double &dyMeters,
    double &dthetaDeg)
{

    if (T.empty())
        return false;

    double a = T.at<double>(0, 0);
    double b = T.at<double>(0, 1);

    double tx = T.at<double>(0, 2);
    double ty = T.at<double>(1, 2);

    double scale =
        std::sqrt(a * a + b * b);

    // Reject obviously broken transforms.
    if (scale < 0.5 || scale > 1.5)
        return false;

    dthetaDeg =
        std::atan2(b, a) *
        180.0 / CV_PI;

    dxMeters =
        tx * metersPerPixel;

    dyMeters =
        ty * metersPerPixel;

    return true;
}

static double calculateConfidence(
    int inliers,
    size_t totalMatches)
{

    if (totalMatches == 0)
        return 0.0;

    double ratio =
        static_cast<double>(inliers) /
        static_cast<double>(totalMatches);

    double countScore =
        std::min(1.0, inliers / 50.0);

    return std::clamp(
        0.5 * countScore +
            0.5 * ratio,
        0.0,
        1.0);
}

class NetworkTablesPublisher
{
public:
    NetworkTablesPublisher(
        const std::string &server)
    {

        nt::NetworkTableInstance inst =
            nt::NetworkTableInstance::GetDefault();

        if (!server.empty())
        {
            inst.SetServer(server);
        }

        inst.StartClient4("JetsonVisualOdometry");

        table =
            inst.GetTable("VisualOdometry");

        xMeters =
            table->GetDoubleTopic("xMeters")
                .Publish();

        yMeters =
            table->GetDoubleTopic("yMeters")
                .Publish();

        thetaDeg =
            table->GetDoubleTopic("thetaDeg")
                .Publish();

        deltaXMeters =
            table->GetDoubleTopic("deltaXMeters")
                .Publish();

        deltaYMeters =
            table->GetDoubleTopic("deltaYMeters")
                .Publish();

        deltaThetaDeg =
            table->GetDoubleTopic("deltaThetaDeg")
                .Publish();

        inliers =
            table->GetIntegerTopic("inliers")
                .Publish();

        confidence =
            table->GetDoubleTopic("confidence")
                .Publish();

        valid =
            table->GetBooleanTopic("valid")
                .Publish();

        timestampUs =
            table->GetIntegerTopic("timestampUs")
                .Publish();

        connected =
            table->GetBooleanTopic("jetsonConnected")
                .Publish(true);
    }

    void publish(
        double x,
        double y,
        double theta,
        double dx,
        double dy,
        double dtheta,
        int inlierCount,
        double conf,
        bool isValid)
    {

        auto now =
            std::chrono::duration_cast<
                std::chrono::microseconds>(
                std::chrono::steady_clock::now()
                    .time_since_epoch())
                .count();

        xMeters.Set(x);
        yMeters.Set(y);
        thetaDeg.Set(theta);

        deltaXMeters.Set(dx);
        deltaYMeters.Set(dy);
        deltaThetaDeg.Set(dtheta);

        inliers.Set(inlierCount);
        confidence.Set(conf);
        valid.Set(isValid);
        timestampUs.Set(now);
        connected.Set(true);
    }

private:
    std::shared_ptr<nt::NetworkTable> table;

    nt::DoublePublisher xMeters;
    nt::DoublePublisher yMeters;
    nt::DoublePublisher thetaDeg;

    nt::DoublePublisher deltaXMeters;
    nt::DoublePublisher deltaYMeters;
    nt::DoublePublisher deltaThetaDeg;

    nt::IntegerPublisher inliers;
    nt::DoublePublisher confidence;

    nt::BooleanPublisher valid;
    nt::IntegerPublisher timestampUs;
    nt::BooleanPublisher connected;
};

int main(int argc, char **argv)
{
    /*
     * Usage:
     *
     * ./frc_vo_nt \
     *     <camera-index> \
     *     <roborio-ip> \
     *     <meters-per-pixel> \
     *     <roi-radius>
     *
     * Example:
     *
     * ./frc_vo_nt 0 10.55.72.2 0.002 300
     */

    int cameraIndex = 0;

    std::string roboRIOIp =
        "10.55.72.2";

    /*
     * PLACEHOLDER.
     *
     * Calibrate this for your camera.
     */
    double metersPerPixel =
        0.002;

    int roiRadius = 300;

    if (argc > 1)
        cameraIndex =
            std::stoi(argv[1]);

    if (argc > 2)
        roboRIOIp =
            argv[2];

    if (argc > 3)
        metersPerPixel =
            std::stod(argv[3]);

    if (argc > 4)
        roiRadius =
            std::stoi(argv[4]);

    std::cout
        << "========== FRC VISUAL ODOMETRY ==========\n"
        << "Camera: " << cameraIndex << "\n"
        << "RoboRIO: " << roboRIOIp << "\n"
        << "NetworkTables: /VisualOdometry\n"
        << "Meters/pixel: " << metersPerPixel << "\n"
        << "ROI radius: " << roiRadius << "\n";

    /*
     * USB camera.
     */
    cv::VideoCapture camera(
        cameraIndex,
        cv::CAP_V4L2);

    if (!camera.isOpened())
        throw std::runtime_error(
            "Could not open USB camera.");

    camera.set(
        cv::CAP_PROP_FRAME_WIDTH,
        640);

    camera.set(
        cv::CAP_PROP_FRAME_HEIGHT,
        480);

    camera.set(
        cv::CAP_PROP_FPS,
        60);

    /*
     * OpenCV pipeline objects are constructed once.
     */
    auto detector =
        createDetector();

    auto descriptor =
        createDescriptor();

    /*
     * NetworkTables client.
     */
    NetworkTablesPublisher ntPublisher(
        roboRIOIp);

    cv::Mat image;
    cv::Mat gray;

    camera >> image;

    if (image.empty())
        throw std::runtime_error(
            "Initial camera frame is empty.");

    if (image.channels() == 3)
        cv::cvtColor(
            image,
            gray,
            cv::COLOR_BGR2GRAY);
    else
        gray = image;

    Frame previous =
        processFrame(
            gray,
            detector,
            descriptor,
            roiRadius);

    double x = 0.0;
    double y = 0.0;
    double thetaDeg = 0.0;

    uint64_t frameNumber = 0;

    while (true)
    {
        auto start =
            std::chrono::steady_clock::now();

        camera >> image;

        if (image.empty())
        {
            ntPublisher.publish(
                x, y, thetaDeg,
                0, 0, 0,
                0, 0,
                false);

            continue;
        }

        if (image.channels() == 3)
            cv::cvtColor(
                image,
                gray,
                cv::COLOR_BGR2GRAY);
        else
            gray = image;

        Frame current =
            processFrame(
                gray,
                detector,
                descriptor,
                roiRadius);

        std::vector<cv::DMatch> matches =
            matchDescriptors(
                previous.descriptors,
                current.descriptors);

        TransformResult result =
            fitTransform(
                previous,
                current,
                matches);

        int inlierCount =
            static_cast<int>(
                result.homographyInliers.size());

        double confidence =
            calculateConfidence(
                inlierCount,
                matches.size());

        double dxMeters = 0.0;
        double dyMeters = 0.0;
        double dthetaDeg = 0.0;

        bool valid =
            result.success &&
            confidence >= 0.35 &&
            extractMotion(
                result.rigidTransform,
                metersPerPixel,
                dxMeters,
                dyMeters,
                dthetaDeg);

        if (valid)
        {
            /*
             * Convert camera/image coordinates into the
             * accumulated world coordinate system.
             *
             * This assumes the camera axes are aligned
             * with the robot axes.
             *
             * If the camera is mounted rotated 90/180 degrees,
             * apply a mounting-axis transform here.
             */
            double thetaRad =
                thetaDeg *
                CV_PI / 180.0;

            double c =
                std::cos(thetaRad);

            double s =
                std::sin(thetaRad);

            x +=
                c * dxMeters -
                s * dyMeters;

            y +=
                s * dxMeters +
                c * dyMeters;

            thetaDeg += dthetaDeg;

            while (thetaDeg > 180.0)
                thetaDeg -= 360.0;

            while (thetaDeg < -180.0)
                thetaDeg += 360.0;
        }
        else
        {
            /*
             * Do not integrate a bad frame.
             */
            dxMeters = 0.0;
            dyMeters = 0.0;
            dthetaDeg = 0.0;
        }

        /*
         * Publish EVERY frame.
         *
         * RoboRIO can decide whether the measurement should
         * actually enter the pose estimator.
         */
        ntPublisher.publish(
            x,
            y,
            thetaDeg,
            dxMeters,
            dyMeters,
            dthetaDeg,
            inlierCount,
            confidence,
            valid);

        auto end =
            std::chrono::steady_clock::now();

        double dt =
            std::chrono::duration<double>(
                end - start)
                .count();

        double fps =
            dt > 0.0 ? 1.0 / dt : 0.0;

        if ((frameNumber++ % 10) == 0)
        {
            std::cout
                << "FPS=" << fps
                << " matches=" << matches.size()
                << " inliers=" << inlierCount
                << " confidence=" << confidence
                << " valid=" << valid
                << " pose=("
                << x << ", "
                << y << ", "
                << thetaDeg
                << ")\n";
        }

        /*
         * Local debug view.
         * Press Q to stop.
         */
        cv::Mat debug =
            current.processed.clone();

        cv::circle(
            debug,
            cv::Point(
                debug.cols / 2,
                debug.rows / 2),
            roiRadius,
            cv::Scalar(255),
            2);

        cv::putText(
            debug,
            "inliers=" +
                std::to_string(inlierCount) +
                " conf=" +
                std::to_string(confidence),
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(255),
            1,
            cv::LINE_AA);

        cv::imshow(
            "FRC Visual Odometry",
            debug);

        if ((cv::waitKey(1) & 0xff) == 'q')
            break;

        previous =
            std::move(current);
    }

    return 0;
}
