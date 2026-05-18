package frc.robot.subsystems.swerve;

import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.GenerateEmptyIO;

/** IO for swerve timestamps */
@GenerateEmptyIO(PhoenixOdometryThread.class)
@NullMarked
public interface SwerveIO {

    /** Inputs for swerve timestamps */
    @AutoLog
    public static class SwerveInputs {
        public double[] timestamps = new double[0];
    }

    /** Update inputs */
    public void updateInputs(SwerveInputs inputs);

    /** Set the ground truth pose. Only useful in sim. */
    public void resetPose(Pose2d pose);

}
