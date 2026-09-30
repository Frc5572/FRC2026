package frc.robot.localization;

import java.util.Optional;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

/** Turret Camera Adapter */
public class TurretCameraAdapter {
    private static final double BUFFER_SECONDS = 1.5;
    private static final double RATE_HALF_WINDOW = 0.04;
    private final Translation3d turretCenter;
    private final TimeInterpolatableBuffer<Rotation2d> angleBuffer =
        TimeInterpolatableBuffer.createBuffer(BUFFER_SECONDS);
    /** Between two samples, a frame counts as whipping if either neighbor was. */
    private final TimeInterpolatableBuffer<Boolean> whippingBuffer =
        TimeInterpolatableBuffer.createBuffer((a, b, t) -> a || b, BUFFER_SECONDS);


    private double lastHubTagTime = Double.NEGATIVE_INFINITY;

    public TurretCameraAdapter(Translation3d turretCenter) {
        this.turretCenter = turretCenter;
    }

    /**
     * Records a turret sample.
     *
     * @param timestamp sample time in seconds
     * @param angle robot-relative turret angle
     * @param whipping whether the turret is swinging the long way around to unwrap
     */
    public void recordTurretAngle(double timestamp, Rotation2d angle, boolean whipping) {
        angleBuffer.addSample(timestamp, angle);
        whippingBuffer.addSample(timestamp, whipping);
    }

    /**
     * Records that a turret-camera frame arrived, whether or not it was accepted for fusion. Ends a
     * turret search once a hub tag is seen.
     *
     * @param arrivalTime loop time the frame was received, in seconds
     * @param sawHubTag whether the frame contained any hub tag
     */
    public void recordFrame(double arrivalTime, boolean sawHubTag) {
        if (sawHubTag) {
            lastHubTagTime = Math.max(lastHubTagTime, arrivalTime);
        }
    }

    /** Loop time the last turret-camera frame containing a hub tag arrived. */
    public double getLastHubTagTime() {
        return lastHubTagTime;
    }

    /** Whether the turret was whipping around at {@code timestamp}. */
    boolean isWhippingAt(double timestamp) {
        return whippingBuffer.getSample(timestamp).orElse(false);
    }

    /**
     * Turret angular rate (rad/s) around {@code timestamp}. Vision latency means samples after the
     * frame are normally available, so this is a central difference.
     */
    double getRateAt(double timestamp) {
        var before = angleBuffer.getSample(timestamp - RATE_HALF_WINDOW);
        var after = angleBuffer.getSample(timestamp + RATE_HALF_WINDOW);
        if (before.isEmpty() || after.isEmpty()) {
            return 0.0;
        }
        return after.get().minus(before.get()).getRadians() / (2 * RATE_HALF_WINDOW);
    }

    Optional<Transform3d> getRobotToCameraAt(Transform3d turretToCamera, double timestamp) {
        var maybeTurretRotation = angleBuffer.getSample(timestamp);
        if (maybeTurretRotation.isEmpty()) {
            return Optional.empty();
        }

        Rotation2d turretAngle = maybeTurretRotation.get();

        Rotation3d turretYaw = new Rotation3d(0.0, 0.0, turretAngle.getRadians());
        Transform3d robotToTurret = new Transform3d(turretCenter, turretYaw);

        Transform3d robotToCamera = robotToTurret.plus(turretToCamera);

        return Optional.of(robotToCamera);
    }
}
