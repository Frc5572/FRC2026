package frc.robot.localization;

import java.util.Optional;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class TurretCameraAdapter {
    private static final double BUFFER_SECONDS = 1.5;
    private final Translation3d turretCenter;
    private final TimeInterpolatableBuffer<Rotation2d> angleBuffer =
        TimeInterpolatableBuffer.createBuffer(BUFFER_SECONDS);


    public TurretCameraAdapter(Translation3d turretCenter) {
        this.turretCenter = turretCenter;
    }

    public void recordTurretAngle(double timestamp, Rotation2d angle) {
        angleBuffer.addSample(timestamp, angle);
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
