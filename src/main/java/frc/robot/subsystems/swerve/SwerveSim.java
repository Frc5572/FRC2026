package frc.robot.subsystems.swerve;

import java.util.stream.Stream;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroSim;
import frc.robot.subsystems.swerve.mod.SwerveModuleIO;
import frc.robot.subsystems.swerve.mod.SwerveModuleSim;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;

/** Simulation implementation for swerve */
@NullMarked
public final class SwerveSim implements SwerveIO {

    private final SwerveModuleSim simModules[] = new SwerveModuleSim[4];
    private Pose2d currentPose;

    /** Simulation implementation for swerve */
    public SwerveSim(Pose2d initialPose) {
        currentPose = initialPose;
    }

    /** Supplier passed into Swerve constructor */
    public SwerveSim simProvider(PhoenixOdometryThread odometryThread) {
        return this;
    }

    /** Supplier passed into Swerve constructor */
    public GyroIO gyroProvider(PhoenixOdometryThread odometryThread) {
        return new GyroSim(this);
    }

    /** Supplier passed into Swerve constructor */
    public SwerveModuleIO moduleProvider(int index, PhoenixOdometryThread odometryThread) {
        simModules[index] = new SwerveModuleSim();
        return simModules[index];
    }

    /** Get ground truth pose. */
    public Pose2d getPose() {
        return currentPose;
    }

    private SwerveModulePosition[] lastWheelPositions =
        new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.timestamps = new double[] {Timer.getTimestamp()};
        var positions = Stream.of(simModules).map(SwerveModuleSim::getPosition)
            .toArray(SwerveModulePosition[]::new);
        Twist2d twist = Constants.Swerve.swerveKinematics.toTwist2d(lastWheelPositions, positions);
        lastWheelPositions = positions;
        currentPose = currentPose.exp(twist);
    }


    @Override
    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

}
