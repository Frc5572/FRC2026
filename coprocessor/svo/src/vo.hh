#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

struct VOPose
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;

    int featureCount = 0;
    int inlierCount = 0;

    double quality = 0.0;
    bool tracking = false;
};

class VisualOdometry
{
public:
    VisualOdometry(
        double fx,
        double fy,
        double cx,
        double cy);

    bool processFrame(
        const cv::Mat &frame,
        double timestamp);

    const VOPose &getPose() const;

    void reset();

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    bool initialized_ = false;

    cv::Mat previousGray_;

    std::vector<cv::Point2f> previousPoints_;

    cv::Mat rotation_;
    cv::Mat translation_;

    VOPose pose_;

    double previousTimestamp_ = 0.0;

    void detectFeatures(
        const cv::Mat &gray);

    bool trackFeatures(
        const cv::Mat &gray,
        std::vector<cv::Point2f> &currentPoints);

    bool estimateMotion(
        const std::vector<cv::Point2f> &previous,
        const std::vector<cv::Point2f> &current);

    void updatePose(
        const cv::Mat &R,
        const cv::Mat &t,
        double dt);

    static double rotationToYaw(
        const cv::Mat &R);
};