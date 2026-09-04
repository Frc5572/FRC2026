#include "VisualOdometry.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <cmath>
#include <iostream>

VisualOdometry::VisualOdometry(
    double fx,
    double fy,
    double cx,
    double cy)
{
    cameraMatrix_ =
        (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
         0.0, fy, cy,
         0.0, 0.0, 1.0);

    distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);

    rotation_ = cv::Mat::eye(3, 3, CV_64F);

    translation_ = cv::Mat::zeros(3, 1, CV_64F);
}

void VisualOdometry::reset()
{
    initialized_ = false;

    previousGray_.release();
    previousPoints_.clear();

    rotation_ = cv::Mat::eye(3, 3, CV_64F);
    translation_ = cv::Mat::zeros(3, 1, CV_64F);

    pose_ = VOPose();

    previousTimestamp_ = 0.0;
}

void VisualOdometry::detectFeatures(
    const cv::Mat &gray)
{
    previousPoints_.clear();

    std::vector<cv::KeyPoint> keypoints;

    cv::FAST(
        gray,
        keypoints,
        20,
        true);

    // Limit the number of points.

    constexpr int MAX_FEATURES = 600;

    if (keypoints.size() > MAX_FEATURES)
    {
        keypoints.resize(MAX_FEATURES);
    }

    previousPoints_.reserve(keypoints.size());

    for (const auto &kp : keypoints)
    {
        previousPoints_.push_back(kp.pt);
    }
}

bool VisualOdometry::trackFeatures(
    const cv::Mat &gray,
    std::vector<cv::Point2f> &currentPoints)
{
    if (previousPoints_.size() < 30)
    {
        return false;
    }

    std::vector<uchar> status;
    std::vector<float> errors;

    cv::calcOpticalFlowPyrLK(
        previousGray_,
        gray,
        previousPoints_,
        currentPoints,
        status,
        errors,
        cv::Size(21, 21),
        3,
        cv::TermCriteria(
            cv::TermCriteria::COUNT |
                cv::TermCriteria::EPS,
            30,
            0.01));

    std::vector<cv::Point2f> goodPrevious;
    std::vector<cv::Point2f> goodCurrent;

    for (size_t i = 0; i < status.size(); ++i)
    {
        if (!status[i])
            continue;

        if (errors[i] > 30.0)
            continue;

        if (currentPoints[i].x < 5 ||
            currentPoints[i].y < 5 ||
            currentPoints[i].x >= gray.cols - 5 ||
            currentPoints[i].y >= gray.rows - 5)
        {
            continue;
        }

        goodPrevious.push_back(previousPoints_[i]);
        goodCurrent.push_back(currentPoints[i]);
    }

    previousPoints_ = goodPrevious;
    currentPoints = goodCurrent;

    return previousPoints_.size() >= 30;
}

bool VisualOdometry::estimateMotion(
    const std::vector<cv::Point2f> &previous,
    const std::vector<cv::Point2f> &current)
{
    if (previous.size() < 30)
    {
        return false;
    }

    cv::Mat mask;

    cv::Mat essential = cv::findEssentialMat(
        previous,
        current,
        cameraMatrix_,
        cv::RANSAC,
        0.999,
        1.0,
        mask);

    if (essential.empty())
    {
        return false;
    }

    cv::Mat R;
    cv::Mat t;

    int inliers = cv::recoverPose(
        essential,
        previous,
        current,
        cameraMatrix_,
        R,
        t,
        mask);

    pose_.inlierCount = inliers;

    if (inliers < 20)
    {
        return false;
    }

    pose_.featureCount =
        static_cast<int>(previous.size());

    pose_.quality =
        static_cast<double>(inliers) /
        static_cast<double>(previous.size());

    double dt =
        1.0 / 60.0;

    updatePose(
        R,
        t,
        dt);

    return true;
}

void VisualOdometry::updatePose(
    const cv::Mat &R,
    const cv::Mat &t,
    double dt)
{
    /*
     * IMPORTANT:
     *
     * recoverPose() gives translation direction,
     * NOT metric translation.
     *
     * Therefore this is a RELATIVE monocular VO
     * trajectory until scale is supplied externally.
     */

    rotation_ = R * rotation_;

    translation_ =
        translation_ +
        rotation_ * t;

    pose_.x = translation_.at<double>(0);
    pose_.y = translation_.at<double>(1);
    pose_.z = translation_.at<double>(2);

    pose_.yaw = rotationToYaw(rotation_);

    pose_.tracking = true;

    if (dt > 0.0)
    {
        pose_.vx =
            pose_.x / dt;

        pose_.vy =
            pose_.y / dt;

        pose_.vz =
            pose_.z / dt;
    }
}

double VisualOdometry::rotationToYaw(
    const cv::Mat &R)
{
    return std::atan2(
        R.at<double>(1, 0),
        R.at<double>(0, 0));
}

bool VisualOdometry::processFrame(
    const cv::Mat &frame,
    double timestamp)
{
    if (frame.empty())
    {
        return false;
    }

    cv::Mat gray;

    if (frame.channels() == 3)
    {
        cv::cvtColor(
            frame,
            gray,
            cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = frame;
    }

    if (!initialized_)
    {
        detectFeatures(gray);

        previousGray_ = gray.clone();

        previousTimestamp_ = timestamp;

        initialized_ = true;

        pose_.featureCount =
            static_cast<int>(
                previousPoints_.size());

        return true;
    }

    double dt =
        timestamp -
        previousTimestamp_;

    if (dt <= 0.0)
    {
        dt = 1.0 / 60.0;
    }

    std::vector<cv::Point2f> currentPoints;

    bool tracked =
        trackFeatures(
            gray,
            currentPoints);

    if (!tracked)
    {
        pose_.tracking = false;

        detectFeatures(gray);

        previousGray_ = gray.clone();

        previousTimestamp_ = timestamp;

        return false;
    }

    bool motion =
        estimateMotion(
            previousPoints_,
            currentPoints);

    /*
     * Replace feature set when necessary.
     */

    if (previousPoints_.size() < 150)
    {
        detectFeatures(gray);
    }
    else
    {
        previousPoints_ = currentPoints;
    }

    previousGray_ = gray.clone();

    previousTimestamp_ = timestamp;

    return motion;
}

const VOPose &VisualOdometry::getPose() const
{
    return pose_;
}