#pragma once

#include <cstdint>
#include <optional>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

class GtsamLocalizer {
public:
    struct Config {
        double odomSigmaX = 0.025;
        double odomSigmaY = 0.025;
        double odomSigmaTheta = 0.02;

        double voSigmaX = 0.05;
        double voSigmaY = 0.05;
        double voSigmaTheta = 0.04;

        double visionSigmaX = 0.20;
        double visionSigmaY = 0.20;
        double visionSigmaTheta = 0.15;
    };

    explicit GtsamLocalizer(const Config& config = {});

    void reset(const gtsam::Pose2& pose, int64_t timestampUs);

    // Add a robot-frame odometry delta from the previous sample to the current sample.
    void addOdometry(const gtsam::Pose2& delta, int64_t timestampUs);

    // Add visual-odometry delta. Confidence is [0, 1]. Invalid measurements are ignored.
    void addVisualOdometry(const gtsam::Pose2& delta, double confidence,
                           int inliers, int64_t timestampUs);

    // Add an absolute field-referenced vision pose, e.g. AprilTag localization.
    void addAbsoluteVision(const gtsam::Pose2& pose, double confidence,
                           int64_t timestampUs);

    gtsam::Pose2 pose() const { return currentPose_; }
    int64_t timestampUs() const { return currentTimestampUs_; }
    bool initialized() const { return initialized_; }
    bool lastVisionAccepted() const { return lastVisionAccepted_; }
    size_t graphSize() const { return graphSize_; }

private:
    gtsam::noiseModel::Diagonal::shared_ptr makeNoise(
        double sigmaX, double sigmaY, double sigmaTheta) const;

    gtsam::noiseModel::Diagonal::shared_ptr scaledVisionNoise(
        double confidence) const;

    void update();
    void addNewPose(const gtsam::Pose2& initialGuess, int64_t timestampUs);

    Config config_;
    gtsam::ISAM2 isam_;
    gtsam::NonlinearFactorGraph pendingGraph_;
    gtsam::Values pendingValues_;

    gtsam::Pose2 currentPose_;
    int64_t currentTimestampUs_ = 0;
    gtsam::Key currentKey_ = 0;
    gtsam::Key previousKey_ = 0;
    size_t graphSize_ = 0;
    bool initialized_ = false;
    bool lastVisionAccepted_ = false;
};
