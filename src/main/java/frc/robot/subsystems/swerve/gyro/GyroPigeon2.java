package frc.robot.subsystems.swerve.gyro;

import java.util.Queue;
import org.jspecify.annotations.NullMarked;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;

/** Pigeon2 implementation for Gyro */
@NullMarked
public class GyroPigeon2 implements GyroIO {

    private Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID, new CANBus(Constants.CANbusName));

    private final Queue<Double> yawQueue;
    private final StatusSignal<Angle> gyroYaw = gyro.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityZDevice();
    private final StatusSignal<Angle> gyroPitch = gyro.getPitch();
    private final StatusSignal<AngularVelocity> pitchVelocity = gyro.getAngularVelocityXDevice();
    private final StatusSignal<Angle> gyroRoll = gyro.getRoll();
    private final StatusSignal<AngularVelocity> rollVelocity = gyro.getAngularVelocityYDevice();

    /** Pigeon2 implementation for Gyro */
    public GyroPigeon2(PhoenixOdometryThread odometryThread) {
        this.yawQueue = odometryThread.registerSignal(gyroYaw);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        BaseStatusSignal.refreshAll(gyroYaw, yawVelocity, gyroPitch, pitchVelocity, gyroRoll,
            rollVelocity);

        inputs.connected = gyro.isConnected();

        double invert = Constants.Swerve.invertGyro ? -1.0 : 1.0;

        inputs.yaw = Rotation2d
            .fromDegrees(invert * gyroYaw.getValue().in(edu.wpi.first.units.Units.Degrees));
        inputs.yawVelocityRadPerSec =
            invert * yawVelocity.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond);
        inputs.pitch = Rotation2d
            .fromDegrees(invert * gyroPitch.getValue().in(edu.wpi.first.units.Units.Degrees));
        inputs.pitchVelocityRadPerSec =
            invert * pitchVelocity.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond);
        inputs.roll = Rotation2d
            .fromDegrees(invert * gyroRoll.getValue().in(edu.wpi.first.units.Units.Degrees));
        inputs.rollVelocityRadPerSec =
            invert * rollVelocity.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond);
        inputs.yawRads = yawQueue.stream().mapToDouble(x -> invert * x).toArray();
        yawQueue.clear();
    }

}
