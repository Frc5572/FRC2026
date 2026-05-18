package frc.robot.subsystems.swerve;

import java.util.Queue;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;

/** Real Swerve Implementation */
@NullMarked
public final class SwerveReal implements SwerveIO {

    private final Queue<Double> timestampQueue;

    /** Real Swerve Implementation */
    public SwerveReal(PhoenixOdometryThread odometryThread) {
        this.timestampQueue = odometryThread.makeTimestampQueue();
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.timestamps = this.timestampQueue.stream().mapToDouble(x -> x).toArray();
        this.timestampQueue.clear();
    }

    @Override
    public void resetPose(Pose2d pose) {}

}
