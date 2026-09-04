package frc.robot.subsystems.localization;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GtsamLocalizer extends SubsystemBase {
  private final GtsamLocalizerIO io;
  private final GtsamLocalizerIOInputsAutoLogged inputs = new GtsamLocalizerIOInputsAutoLogged();

  public GtsamLocalizer(GtsamLocalizerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GtsamLocalizer", inputs);

    Logger.recordOutput("GtsamLocalizer/Pose", getPose());
    Logger.recordOutput("GtsamLocalizer/Valid", inputs.valid);
    Logger.recordOutput("GtsamLocalizer/VisionAccepted", inputs.visionAccepted);
    Logger.recordOutput("GtsamLocalizer/GraphSize", inputs.graphSize);
  }

  public Pose2d getPose() {
    return new Pose2d(inputs.xMeters, inputs.yMeters, new Rotation2d(inputs.thetaRadians));
  }

  public boolean isValid() {
    return inputs.valid;
  }

  public boolean visionAccepted() {
    return inputs.visionAccepted;
  }

  public long getTimestampUs() {
    return inputs.timestampUs;
  }

  public void addOdometryDelta(Pose2d delta, long timestampUs) {
    io.setOdometryDelta(delta, timestampUs);
  }

  public void resetPose(Pose2d pose, long timestampUs) {
    io.resetPose(pose, timestampUs);
  }
}
