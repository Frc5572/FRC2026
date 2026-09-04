#pragma once

#include "VisualOdometry.hpp"

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <memory>

class NetworkTablesPublisher
{
public:
    NetworkTablesPublisher();

    void publish(
        const VOPose &pose,
        double timestamp);

private:
    std::shared_ptr<nt::NetworkTable> table_;
};