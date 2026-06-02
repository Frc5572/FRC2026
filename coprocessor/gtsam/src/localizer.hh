#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <exception>
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

        oldOdomPose_ = Pose2(0.0, 0.0, 0.0);
        oldVisionPose_ = Pose2(0.0, 0.0, 0.0);
    }

    Pose2 addOdometry(const Pose2 &odomDelta)
    {
        if (!std::isfinite(odomDelta.x()) || !std::isfinite(odomDelta.y()) || !std::isfinite(odomDelta.theta()) || oldOdomPose_.equals(odomDelta, tol_) || (std::hypot(odomDelta.x(), odomDelta.y()) < 0.05 && std::abs(odomDelta.theta()) < 0.035))
        {
            return getLatestPose();
        }

        if (currentIndex_ >= maxStates_)
        {
            resetGraph(latestPose_);
        }

        NonlinearFactorGraph newFactors;
        Values newValues;

        const size_t nextIndex = currentIndex_ + 1;

        auto odomNoise = Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.10, 0.10, 0.10).finished());

        newFactors.add(BetweenFactor<Pose2>(
            X(currentIndex_),
            X(nextIndex),
            odomDelta,
            odomNoise));

        Pose2 initialGuess = latestPose_.compose(odomDelta);
        newValues.insert(X(nextIndex), initialGuess);

        try
        {
            isam_.update(newFactors, newValues);

            currentIndex_ = nextIndex;
            latestPose_ = isam_.calculateEstimate<Pose2>(X(currentIndex_));
        }
        catch (const std::exception &e)
        {
            std::cout << "GTSAM odom update failed at key=" << nextIndex
                      << ": " << e.what() << "\n";
            resetGraph(latestPose_);
            return latestPose_;
        }

        oldOdomPose_ = odomDelta;

        // std::cout << "ODOM key=" << currentIndex_
        //           << " pose=" << latestPose_ << "\n";

        return latestPose_;
    }

    Pose2 addVisionMeasurement(const Pose2 &cameraFieldPose, double translationStdDev, double rotStdDev)
    {
        if (!std::isfinite(cameraFieldPose.x()) || !std::isfinite(cameraFieldPose.y()) || !std::isfinite(cameraFieldPose.theta()) || !std::isfinite(translationStdDev) || !std::isfinite(rotStdDev) || translationStdDev <= 1e-6 || rotStdDev <= 1e-6 || cameraFieldPose.equals(oldVisionPose_, 1e-3))
        {
            return getLatestPose();
        }
        NonlinearFactorGraph newFactors;
        Values noNewValues;

        auto visionNoise = Diagonal::Sigmas(
            (gtsam::Vector(3) << translationStdDev, translationStdDev, rotStdDev).finished());

        newFactors.add(PriorFactor<Pose2>(
            X(currentIndex_),
            cameraFieldPose,
            visionNoise));

        if (!inited_)
        {
            inited_ = true;
            resetPose(cameraFieldPose);
        }
        try
        {
            isam_.update(newFactors, noNewValues);

            latestPose_ = isam_.calculateEstimate<Pose2>(X(currentIndex_));
        }
        catch (const std::exception &e)
        {
            std::cout << "GTSAM vision update failed at key=" << currentIndex_
                      << ": " << e.what() << "\n";
            resetGraph(latestPose_);
            return latestPose_;
        }

        // std::cout << "VISION key=" << currentIndex_
        //           << " measurement=" << cameraFieldPose
        //           << " pose=" << latestPose_ << "\n";

        oldVisionPose_ = cameraFieldPose;
        return latestPose_;
    }

    bool isInited()
    {
        return inited_;
    }

    Pose2 getLatestPose() const
    {
        return latestPose_;
    }

    Pose2 resetPose(const Pose2 &fieldPose)
    {
        if (!std::isfinite(fieldPose.x()) || !std::isfinite(fieldPose.y()) || !std::isfinite(fieldPose.theta()))
        {
            return getLatestPose();
        }

        NonlinearFactorGraph newFactors;
        Values newValues;

        const size_t nextIndex = currentIndex_ + 1;

        auto odomNoise = Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.01, 0.01, 0.01).finished());

        newFactors.add(BetweenFactor<Pose2>(
            X(currentIndex_),
            X(nextIndex),
            latestPose_.between(fieldPose),
            odomNoise));

        newValues.insert(X(nextIndex), fieldPose);

        try
        {
            isam_.update(newFactors, newValues);

            currentIndex_ = nextIndex;
            latestPose_ = isam_.calculateEstimate<Pose2>(X(currentIndex_));
        }
        catch (const std::exception &e)
        {
            std::cout << "GTSAM reset failed at key=" << nextIndex
                      << ": " << e.what() << "\n";
            resetGraph(latestPose_);
            return latestPose_;
        }

        oldOdomPose_ = Pose2(0.0, 0.0, 0.0);

        std::cout << "ODOM key=" << currentIndex_
                  << " pose=" << latestPose_ << "\n";

        return latestPose_;
    }

    gtsam::Pose2 resetTranslation(
        double newX,
        double newY)
    {
        return resetPose(gtsam::Pose2(newX, newY, latestPose_.theta()));
    }

private:
    void resetGraph(const Pose2 &pose)
    {
        ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        isam_ = ISAM2(params);

        auto resetNoise = Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.05, 0.05, 0.05).finished());

        NonlinearFactorGraph graph;
        Values values;
        graph.add(PriorFactor<Pose2>(X(0), pose, resetNoise));
        values.insert(X(0), pose);

        isam_.update(graph, values);
        currentIndex_ = 0;
        latestPose_ = pose;
        oldOdomPose_ = Pose2(0.0, 0.0, 0.0);
        oldVisionPose_ = Pose2(0.0, 0.0, 0.0);
    }

    ISAM2 isam_;
    static constexpr size_t maxStates_ = 75;
    size_t currentIndex_ = 0;
    Pose2 latestPose_;
    bool inited_ = false;
    Pose2 oldOdomPose_;
    Pose2 oldVisionPose_;
    double tol_ = 1e-8;
};
