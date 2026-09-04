#include "GtsamLocalizer.h"

#include <algorithm>
#include <cmath>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

GtsamLocalizer::GtsamLocalizer(const Config& config)
    : config_(config), isam_(gtsam::ISAM2Params()) {}

gtsam::noiseModel::Diagonal::shared_ptr GtsamLocalizer::makeNoise(
    double sigmaX, double sigmaY, double sigmaTheta) const {
    return gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << sigmaX, sigmaY, sigmaTheta).finished());
}

gtsam::noiseModel::Diagonal::shared_ptr GtsamLocalizer::scaledVisionNoise(
    double confidence) const {
    // Confidence 1.0 = configured covariance.
    // Lower confidence increases sigma, causing GTSAM to trust the factor less.
    const double c = std::clamp(confidence, 0.05, 1.0);
    const double scale = 1.0 / std::sqrt(c);
    return makeNoise(
        config_.visionSigmaX * scale,
        config_.visionSigmaY * scale,
        config_.visionSigmaTheta * scale);
}

void GtsamLocalizer::reset(const gtsam::Pose2& pose, int64_t timestampUs) {
    // Rebuild ISAM2. This is intentional: a reset should discard the old graph.
    isam_ = gtsam::ISAM2(gtsam::ISAM2Params());
    pendingGraph_.resize(0);
    pendingValues_.clear();

    currentKey_ = gtsam::Symbol('x', 0);
    previousKey_ = currentKey_;
    currentPose_ = pose;
    currentTimestampUs_ = timestampUs;
    graphSize_ = 0;
    initialized_ = true;
    lastVisionAccepted_ = false;

    auto priorNoise = makeNoise(0.001, 0.001, 0.001);
    pendingGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(
        currentKey_, pose, priorNoise));
    pendingValues_.insert(currentKey_, pose);
    update();
}

void GtsamLocalizer::addNewPose(const gtsam::Pose2& initialGuess,
                                int64_t timestampUs) {
    previousKey_ = currentKey_;
    ++currentKey_;
    pendingValues_.insert(currentKey_, initialGuess);
    currentTimestampUs_ = timestampUs;
}

void GtsamLocalizer::addOdometry(const gtsam::Pose2& delta,
                                 int64_t timestampUs) {
    if (!initialized_) {
        reset(gtsam::Pose2(), timestampUs);
        return;
    }

    const gtsam::Pose2 guess = currentPose_.compose(delta);
    addNewPose(guess, timestampUs);

    pendingGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
        previousKey_, currentKey_, delta,
        makeNoise(config_.odomSigmaX,
                  config_.odomSigmaY,
                  config_.odomSigmaTheta)));

    update();
}

void GtsamLocalizer::addVisualOdometry(const gtsam::Pose2& delta,
                                       double confidence,
                                       int inliers,
                                       int64_t timestampUs) {
    lastVisionAccepted_ = false;

    if (!initialized_ || confidence < 0.35 || inliers < 5) {
        return;
    }

    // VO is another relative motion measurement between the current pose nodes.
    // We attach it to the newest pose pair. If no odometry sample created a new
    // node yet, create one here.
    if (currentTimestampUs_ == timestampUs) {
        return;
    }

    const gtsam::Pose2 guess = currentPose_.compose(delta);
    addNewPose(guess, timestampUs);

    const double c = std::clamp(confidence, 0.05, 1.0);
    const double scale = 1.0 / std::sqrt(c);
    pendingGraph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
        previousKey_, currentKey_, delta,
        makeNoise(config_.voSigmaX * scale,
                  config_.voSigmaY * scale,
                  config_.voSigmaTheta * scale)));

    lastVisionAccepted_ = true;
    update();
}

void GtsamLocalizer::addAbsoluteVision(const gtsam::Pose2& pose,
                                       double confidence,
                                       int64_t timestampUs) {
    lastVisionAccepted_ = false;

    if (!initialized_ || confidence < 0.35) {
        return;
    }

    // Absolute vision constrains the current estimate. We don't create a new
    // state just for the measurement, because this factor represents a pose
    // observation at the latest available graph state.
    pendingGraph_.add(gtsam::PriorFactor<gtsam::Pose2>(
        currentKey_, pose, scaledVisionNoise(confidence)));

    currentTimestampUs_ = std::max(currentTimestampUs_, timestampUs);
    lastVisionAccepted_ = true;
    update();
}

void GtsamLocalizer::update() {
    if (pendingGraph_.empty() && pendingValues_.empty()) {
        return;
    }

    isam_.update(pendingGraph_, pendingValues_);
    pendingGraph_.resize(0);
    pendingValues_.clear();

    const gtsam::Values estimate = isam_.calculateEstimate();
    if (estimate.exists<gtsam::Pose2>(currentKey_)) {
        currentPose_ = estimate.at<gtsam::Pose2>(currentKey_);
    }

    graphSize_++;
}
