#include "NetworkTablesIO.h"

NetworkTablesIO::NetworkTablesIO(nt::NetworkTableInstance inst) {
    table_ = inst.GetTable("GtsamLocalizer");

    auto input = table_->GetSubTable("Input");
    auto vo = table_->GetSubTable("VisualOdometry");
    auto vision = table_->GetSubTable("Vision");
    auto reset = table_->GetSubTable("Reset");
    auto output = table_->GetSubTable("Output");

    odomDx_ = input->GetDoubleTopic("DeltaX").Subscribe(0.0);
    odomDy_ = input->GetDoubleTopic("DeltaY").Subscribe(0.0);
    odomDtheta_ = input->GetDoubleTopic("DeltaTheta").Subscribe(0.0);
    odomTimestamp_ = input->GetIntegerTopic("TimestampUs").Subscribe(0);
    odomValid_ = input->GetBooleanTopic("Valid").Subscribe(false);

    voDx_ = vo->GetDoubleTopic("DeltaX").Subscribe(0.0);
    voDy_ = vo->GetDoubleTopic("DeltaY").Subscribe(0.0);
    voDtheta_ = vo->GetDoubleTopic("DeltaTheta").Subscribe(0.0);
    voConfidence_ = vo->GetDoubleTopic("Confidence").Subscribe(0.0);
    voInliers_ = vo->GetIntegerTopic("Inliers").Subscribe(0);
    voTimestamp_ = vo->GetIntegerTopic("TimestampUs").Subscribe(0);
    voValid_ = vo->GetBooleanTopic("Valid").Subscribe(false);

    visionX_ = vision->GetDoubleTopic("X").Subscribe(0.0);
    visionY_ = vision->GetDoubleTopic("Y").Subscribe(0.0);
    visionTheta_ = vision->GetDoubleTopic("Theta").Subscribe(0.0);
    visionConfidence_ = vision->GetDoubleTopic("Confidence").Subscribe(0.0);
    visionTimestamp_ = vision->GetIntegerTopic("TimestampUs").Subscribe(0);
    visionValid_ = vision->GetBooleanTopic("Valid").Subscribe(false);

    reset_ = reset->GetBooleanTopic("Request").Subscribe(false);
    resetX_ = reset->GetDoubleTopic("X").Subscribe(0.0);
    resetY_ = reset->GetDoubleTopic("Y").Subscribe(0.0);
    resetTheta_ = reset->GetDoubleTopic("Theta").Subscribe(0.0);
    resetTimestamp_ = reset->GetIntegerTopic("TimestampUs").Subscribe(0);

    outputX_ = output->GetDoubleTopic("X").Publish();
    outputY_ = output->GetDoubleTopic("Y").Publish();
    outputTheta_ = output->GetDoubleTopic("Theta").Publish();
    outputTimestamp_ = output->GetIntegerTopic("TimestampUs").Publish();
    outputValid_ = output->GetBooleanTopic("Valid").Publish();
    visionAccepted_ = output->GetBooleanTopic("VisionAccepted").Publish();
    graphSize_ = output->GetIntegerTopic("GraphSize").Publish();
    connected_ = output->GetBooleanTopic("Connected").Publish();
}

NetworkTablesIO::Inputs NetworkTablesIO::read() {
    Inputs i;
    i.odomDx = odomDx_.Get();
    i.odomDy = odomDy_.Get();
    i.odomDtheta = odomDtheta_.Get();
    i.odomTimestampUs = odomTimestamp_.Get();
    i.odomValid = odomValid_.Get();

    i.voDx = voDx_.Get();
    i.voDy = voDy_.Get();
    i.voDtheta = voDtheta_.Get();
    i.voConfidence = voConfidence_.Get();
    i.voInliers = static_cast<int>(voInliers_.Get());
    i.voTimestampUs = voTimestamp_.Get();
    i.voValid = voValid_.Get();

    i.visionX = visionX_.Get();
    i.visionY = visionY_.Get();
    i.visionTheta = visionTheta_.Get();
    i.visionConfidence = visionConfidence_.Get();
    i.visionTimestampUs = visionTimestamp_.Get();
    i.visionValid = visionValid_.Get();

    i.reset = reset_.Get();
    i.resetX = resetX_.Get();
    i.resetY = resetY_.Get();
    i.resetTheta = resetTheta_.Get();
    i.resetTimestampUs = resetTimestamp_.Get();
    return i;
}

void NetworkTablesIO::publish(double x, double y, double theta,
                              int64_t timestampUs, bool valid,
                              bool visionAccepted, size_t graphSize) {
    outputX_.Set(x);
    outputY_.Set(y);
    outputTheta_.Set(theta);
    outputTimestamp_.Set(timestampUs);
    outputValid_.Set(valid);
    visionAccepted_.Set(visionAccepted);
    graphSize_.Set(static_cast<int64_t>(graphSize));
    connected_.Set(true);
}
