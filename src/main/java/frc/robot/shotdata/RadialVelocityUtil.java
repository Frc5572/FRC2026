package frc.robot.shotdata;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RadialVelocityUtil {

    /** Result: distance, radial velocity, and tangential velocity vector (field frame). */
    public static record MotionResult(double distanceMeters, double radialVelocityMetersPerSecond,
        Translation2d tangentialVelocityField) {
    }

    /**
     * Computes:
     * <ul>
     * <li>distance from a target to an arbitrary point on the robot</li>
     * <li>radial velocity of that point relative to the target</li>
     * <li>tangential velocity vector of that point relative to the target (field frame)</li>
     * </ul>
     *
     * Conventions:
     * <ul>
     * <li>distanceMeters: |target -> point|</li>
     * <li>radialVelocityMetersPerSecond: > 0 => moving away from target along line of sight < 0 =>
     * moving toward target</li>
     * <li>tangentialVelocityField: Vector in field coords representing only the component
     * perpendicular to the line-of-sight (no radial component).</li>
     * </ul>
     *
     * @param chassisSpeeds Robot chassis speeds (field-relative vx, vy, omega). All shooting math is
     *        done in field coordinates; robot-relative conversion is each subsystem's concern.
     * @param robotPose Robot pose in the field frame.
     * @param pointOffsetRobot Offset of the point from the robot center in robot frame.
     * @param targetPositionField Target position in the field frame.
     */
    public static MotionResult getDistanceRadialTangentialVector(ChassisSpeeds chassisSpeeds,
        Pose2d robotPose, Translation2d pointOffsetRobot, Translation2d targetPositionField) {

        Rotation2d heading = robotPose.getRotation();

        // 1) Robot linear velocity in field frame (chassisSpeeds is already field-relative)
        double vxField = chassisSpeeds.vxMetersPerSecond;
        double vyField = chassisSpeeds.vyMetersPerSecond;

        // 2) Point position in field frame (robot center + rotated offset)
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d offsetField = pointOffsetRobot.rotateBy(heading);
        Translation2d pointPosField = robotTranslation.plus(offsetField);

        // 3) Rotational velocity component of that point in field frame
        double omega = chassisSpeeds.omegaRadiansPerSecond;
        double rx = offsetField.getX();
        double ry = offsetField.getY();

        double vxRot = -omega * ry;
        double vyRot = omega * rx;

        // 4) Total velocity of the point in field frame
        double vxPointField = vxField + vxRot;
        double vyPointField = vyField + vyRot;

        // 5) Vector from target to point
        double dx = pointPosField.getX() - targetPositionField.getX();
        double dy = pointPosField.getY() - targetPositionField.getY();
        double dist = Math.hypot(dx, dy);

        if (dist < 1e-9) {
            // Point essentially at the target
            return new MotionResult(0.0, 0.0, new Translation2d());
        }

        // Unit radial vector (target -> point)
        double ux = dx / dist;
        double uy = dy / dist;

        // Radial velocity scalar
        double radialVel = vxPointField * ux + vyPointField * uy;

        // Full velocity vector
        double vx = vxPointField;
        double vy = vyPointField;

        // Radial component vector = (v · u) * u
        double vxRadial = radialVel * ux;
        double vyRadial = radialVel * uy;

        // Tangential component vector = v - v_radial
        double vxTangential = vx - vxRadial;
        double vyTangential = vy - vyRadial;

        Translation2d tangentialVelField = new Translation2d(vxTangential, vyTangential);

        return new MotionResult(dist, radialVel, tangentialVelField);
    }

}
