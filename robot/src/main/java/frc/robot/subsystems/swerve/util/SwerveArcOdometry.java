package frc.robot.subsystems.swerve.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Swerve drive odometry implementation that integrates wheel motion using circular arc geometry
 * rather than straight-line approximations.
 *
 * <p>
 * Inspired by Team 1690, this odometry model treats each swerve module's motion between updates as
 * an arc defined by the change in wheel angle and distance traveled. This more accurately
 * represents real swerve motion, especially during simultaneous translation and rotation, where
 * straight-line assumptions can introduce measurable integration error.
 *
 * <p>
 * Compared to {@link SwerveDriveOdometry}, which assumes each module travels in a straight line
 * over a timestep, this class:
 * <ul>
 * <li>Computes per-module displacement along a circular arc</li>
 * <li>Transforms those displacements into the field frame</li>
 * <li>Averages the resulting module displacements to update the robot pose</li>
 * </ul>
 *
 * <p>
 * The robot heading is still sourced directly from the gyro, while translation is derived purely
 * from module motion.
 *
 * <p>
 * This class is intended as a drop-in replacement for {@link SwerveDriveOdometry} when improved
 * accuracy is desired at the cost of slightly increased computation.
 */
public final class SwerveArcOdometry extends SwerveDriveOdometry {
    private final int numberOfModules;
    private final Translation2d[] robotRelativeModuleOffsets;

    private Pose2d robotPose = Pose2d.kZero;
    private final SwerveModulePosition[] previousWheelPositions;

    /**
     * Constructs a new arc-based swerve odometry instance.
     *
     * @param kinematics the swerve drive kinematics describing module locations
     * @param gyroAngle the initial robot heading from the gyro
     * @param modulePositions the initial positions of each swerve module
     */
    public SwerveArcOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions) {
        super(kinematics, gyroAngle, modulePositions);
        numberOfModules = kinematics.getModules().length;
        robotRelativeModuleOffsets = kinematics.getModules();

        previousWheelPositions = new SwerveModulePosition[numberOfModules];
        for (int i = 0; i < numberOfModules; i++) {
            previousWheelPositions[i] = new SwerveModulePosition();
        }
    }

    /**
     * Computes the 2D displacement of a swerve module by integrating its motion along a circular
     * arc.
     *
     * <p>
     * The arc is defined by the change in module steering angle and the change in wheel distance
     * between two samples. If the steering angle does not change, the motion is treated as a
     * straight line for efficiency.
     *
     * <p>
     * This calculation assumes the module follows a constant-curvature path over the timestep,
     * which more closely matches real swerve behavior during rotation.
     *
     * @param previousWheelPosition the module position at the previous update
     * @param currentWheelPosition the module position at the current update
     * @return the module's displacement in the robot-relative frame
     */
    private static Translation2d getModuleDisplacement(SwerveModulePosition previousWheelPosition,
        SwerveModulePosition currentWheelPosition) {
        // First, calculate difference between previous and current angles and distances
        double angleDifferenceRadians =
            currentWheelPosition.angle.getRadians() - previousWheelPosition.angle.getRadians();
        double arcLength =
            currentWheelPosition.distanceMeters - previousWheelPosition.distanceMeters;

        // *If angle difference is 0 then we can just use a straight line instead of an arc
        if (angleDifferenceRadians == 0) {
            return new Translation2d(arcLength, currentWheelPosition.angle);
        }

        // Next, calculate radius. Positive = left turn, negative = right turn
        double radius = (arcLength / angleDifferenceRadians);

        // Then, calculate the center point of the circle that the arc is a part of, using the
        // previous
        // angle. The previous module translation is (0, 0) because we don't care where it starts,
        // only
        // the displacement. It is also always perpendicular to the previous angle, with
        // positive/negative radius indicating which side of the module that the circle center will
        // be
        // on
        double circleCenterX = -radius * previousWheelPosition.angle.getSin();
        double circleCenterY = radius * previousWheelPosition.angle.getCos();

        // Finally, calculate the current module translation on the arc and return it as module
        // displacement
        double displacementX = circleCenterX + radius * currentWheelPosition.angle.getSin();
        double displacementY = circleCenterY - radius * currentWheelPosition.angle.getCos();

        return new Translation2d(displacementX, displacementY);
    }

    @Override
    public Pose2d update(Rotation2d currentGyroAngle,
        SwerveModulePosition[] currentWheelPositions) {
        // First, get the field relative module poses of the previous robot pose, and apply robot
        // relative module offsets
        Pose2d[] fieldRelativeModulePosesOfPreviousPose = new Pose2d[numberOfModules];
        for (int i = 0; i < numberOfModules; i++) {
            fieldRelativeModulePosesOfPreviousPose[i] = robotPose
                .transformBy(new Transform2d(robotRelativeModuleOffsets[i], Rotation2d.kZero));
        }

        // Also get the module displacements from the previous wheel positions to the current wheel
        // positions
        Translation2d[] moduleDisplacements = new Translation2d[numberOfModules];
        for (int i = 0; i < numberOfModules; i++) {
            moduleDisplacements[i] =
                getModuleDisplacement(previousWheelPositions[i], currentWheelPositions[i]);
        }

        // Next, add the module displacements to the field relative module poses
        Translation2d[] fieldRelativeModuleDisplacements = new Translation2d[numberOfModules];
        for (int i = 0; i < numberOfModules; i++) {
            fieldRelativeModuleDisplacements[i] = fieldRelativeModulePosesOfPreviousPose[i]
                .transformBy(new Transform2d(moduleDisplacements[i], Rotation2d.kZero))
                .getTranslation();
        }

        // Finally, average the module displacements and return the new pose
        Translation2d sumOfFieldRelativeModuleDisplacements = new Translation2d();
        for (int i = 0; i < numberOfModules; i++) {
            sumOfFieldRelativeModuleDisplacements =
                sumOfFieldRelativeModuleDisplacements.plus(fieldRelativeModuleDisplacements[i]);
        }
        double updatedPoseX = sumOfFieldRelativeModuleDisplacements.getX() / numberOfModules;
        double updatedPoseY = sumOfFieldRelativeModuleDisplacements.getY() / numberOfModules;
        var updatedPose = new Pose2d(updatedPoseX, updatedPoseY, currentGyroAngle);

        // After calculations, but before the next loop, update the previous pose & wheel positions
        // to
        // the current ones
        robotPose = updatedPose;
        for (int i = 0; i < numberOfModules; i++) {
            previousWheelPositions[i] = currentWheelPositions[i];
        }

        return updatedPose;
    }

    @Override
    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions,
        Pose2d poseMeters) {
        robotPose = new Pose2d(poseMeters.getTranslation(), gyroAngle);
    }

    @Override
    public void resetPose(Pose2d poseMeters) {
        robotPose = poseMeters;
    }

    @Override
    public void resetTranslation(Translation2d translation) {
        robotPose = new Pose2d(translation, robotPose.getRotation());
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        robotPose = new Pose2d(robotPose.getTranslation(), rotation);
    }

    @Override
    public Pose2d getPoseMeters() {
        return robotPose;
    }

}
