package frc.robot.subsystems.swerve.gyro;

import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.GenerateEmptyIO;

/** IO for gyro */
@GenerateEmptyIO({PhoenixOdometryThread.class})
@NullMarked
public interface GyroIO {

    /** Inputs for gyro */
    @AutoLog
    public static class GyroInputs {
        public boolean connected;
        public Rotation2d yaw = Rotation2d.kZero;
        public double yawVelocityRadPerSec;
        public Rotation2d pitch = Rotation2d.kZero;
        public double pitchVelocityRadPerSec;
        public Rotation2d roll = Rotation2d.kZero;
        public double rollVelocityRadPerSec;

        public double[] yawRads = new double[0];
    }

    /** Update inputs */
    public void updateInputs(GyroInputs inputs);

}
