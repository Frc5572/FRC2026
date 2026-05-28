#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

using gtsam::BetweenFactor;
using gtsam::ISAM2;
using gtsam::ISAM2Params;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose2;
using gtsam::PriorFactor;
using gtsam::Values;
using gtsam::noiseModel::Diagonal;
using gtsam::symbol_shorthand::X;

class IsamLocalizer
{
public:
    IsamLocalizer()
    {
        ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        isam_ = ISAM2(params);

        auto startNoise = Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.05, 0.05, 0.02).finished());

        NonlinearFactorGraph graph;
        Values values;

        latestPose_ = Pose2(0.0, 0.0, 0.0);

        graph.add(PriorFactor<Pose2>(X(0), latestPose_, startNoise));
        values.insert(X(0), latestPose_);

        isam_.update(graph, values);
        currentIndex_ = 0;
    }

    Pose2 addOdometry(const Pose2 &odomDelta)
    {
        NonlinearFactorGraph newFactors;
        Values newValues;

        const size_t nextIndex = currentIndex_ + 1;

        auto odomNoise = Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.10, 0.10, 0.04).finished());

        newFactors.add(BetweenFactor<Pose2>(
            X(currentIndex_),
            X(nextIndex),
            odomDelta,
            odomNoise));

        Pose2 initialGuess = latestPose_.compose(odomDelta);
        newValues.insert(X(nextIndex), initialGuess);

        isam_.update(newFactors, newValues);

        currentIndex_ = nextIndex;
        latestPose_ = isam_.calculateEstimate<Pose2>(X(currentIndex_));

        std::cout << "ODOM key=" << currentIndex_
                  << " pose=" << latestPose_ << "\n";

        return latestPose_;
    }

    Pose2 addVisionMeasurement(const Pose2 &cameraFieldPose)
    {
        NonlinearFactorGraph newFactors;
        Values noNewValues;

        auto visionNoise = Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.08, 0.08, 2.00).finished());

        newFactors.add(PriorFactor<Pose2>(
            X(currentIndex_),
            cameraFieldPose,
            visionNoise));

        isam_.update(newFactors, noNewValues);

        latestPose_ = isam_.calculateEstimate<Pose2>(X(currentIndex_));

        std::cout << "VISION key=" << currentIndex_
                  << " measurement=" << cameraFieldPose
                  << " pose=" << latestPose_ << "\n";

        return latestPose_;
    }

    Pose2 getLatestPose() const
    {
        return latestPose_;
    }

private:
    ISAM2 isam_;
    size_t currentIndex_ = 0;
    Pose2 latestPose_;
};