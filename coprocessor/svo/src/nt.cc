#include "NetworkTablesPublisher.hpp"

NetworkTablesPublisher::NetworkTablesPublisher()
{
    auto instance =
        nt::NetworkTableInstance::GetDefault();

    table_ =
        instance.GetTable("SVO");

    instance.StartClient4("JetsonSVO");

    /*
     * Change this to the IP address of your roboRIO.
     */

    instance.SetServer(
        "10.55.72.2");
}

void NetworkTablesPublisher::publish(
    const VOPose &pose,
    double timestamp)
{
    table_->PutNumber(
        "X",
        pose.x);

    table_->PutNumber(
        "Y",
        pose.y);

    table_->PutNumber(
        "Z",
        pose.z);

    table_->PutNumber(
        "Yaw",
        pose.yaw);

    table_->PutNumber(
        "VX",
        pose.vx);

    table_->PutNumber(
        "VY",
        pose.vy);

    table_->PutNumber(
        "VZ",
        pose.vz);

    table_->PutNumber(
        "FeatureCount",
        pose.featureCount);

    table_->PutNumber(
        "InlierCount",
        pose.inlierCount);

    table_->PutNumber(
        "Quality",
        pose.quality);

    table_->PutBoolean(
        "Tracking",
        pose.tracking);

    table_->PutNumber(
        "Timestamp",
        timestamp);
}