package frc.robot.subsystems.swerve.gyro;

import java.util.Queue;
import org.jspecify.annotations.NullMarked;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;

/** Pigeon2 implementation for Gyro */
@NullMarked
public class GyroPigeon2 implements GyroIO {

    private Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID);

    private final Queue<Double> yawQueue;

    /** Pigeon2 implementation for Gyro */
    public GyroPigeon2(PhoenixOdometryThread odometryThread) {
        this.yawQueue = odometryThread
            .registerSignal(() -> gyro.getYaw().getValue().in(edu.wpi.first.units.Units.Radians));
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = gyro.isConnected();

        double invert = Constants.Swerve.invertGyro ? -1.0 : 1.0;

        inputs.yaw = Rotation2d
            .fromDegrees(invert * gyro.getYaw().getValue().in(edu.wpi.first.units.Units.Degrees));
        inputs.yawVelocityRadPerSec = invert * gyro.getAngularVelocityZDevice().getValue()
            .in(edu.wpi.first.units.Units.RadiansPerSecond);
        inputs.pitch = Rotation2d
            .fromDegrees(invert * gyro.getPitch().getValue().in(edu.wpi.first.units.Units.Degrees));
        inputs.pitchVelocityRadPerSec = invert * gyro.getAngularVelocityXDevice().getValue()
            .in(edu.wpi.first.units.Units.RadiansPerSecond);
        inputs.roll = Rotation2d
            .fromDegrees(invert * gyro.getRoll().getValue().in(edu.wpi.first.units.Units.Degrees));
        inputs.rollVelocityRadPerSec = invert * gyro.getAngularVelocityZDevice().getValue()
            .in(edu.wpi.first.units.Units.RadiansPerSecond);
        inputs.yawRads = yawQueue.stream().mapToDouble(x -> invert * x).toArray();
        yawQueue.clear();
    }

}
