package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface GtsamLocalizerIO {
  @AutoLog
  class GtsamLocalizerIOInputs {
    public double xMeters = 0.0;
    public double yMeters = 0.0;
    public double thetaRadians = 0.0;
    public long timestampUs = 0L;

    public boolean valid = false;
    public boolean visionAccepted = false;
    public long graphSize = 0L;

    public double positionStdDev = 0.0;
    public double headingStdDev = 0.0;
  }

  default void updateInputs(GtsamLocalizerIOInputs inputs) {}

  default void setOdometryDelta(Pose2d delta, long timestampUs) {}

  default void resetPose(Pose2d pose, long timestampUs) {}
}
