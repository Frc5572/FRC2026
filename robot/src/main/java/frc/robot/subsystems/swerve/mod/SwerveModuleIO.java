package frc.robot.subsystems.swerve.mod;

import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.GenerateEmptyIO;

/** IO for swerve module */
@GenerateEmptyIO({int.class, PhoenixOdometryThread.class})
@NullMarked
public interface SwerveModuleIO {

    /** Inputs for swerve module */
    @AutoLog
    public static class SwerveModuleInputs {
        public boolean driveConnected;
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveAppliedVolts;
        public double driveSupplyCurrentAmps;
        public double driveStatorCurrentAmps;

        public boolean angleConnected;
        public Rotation2d anglePosition = Rotation2d.kZero;
        public double angleVelocityRadPerSec;
        public double angleAppliedVolts;
        public double angleSupplyCurrentAmps;
        public double angleStatorCurrentAmps;

        public boolean absoluteAngleConnected;
        public Rotation2d angleAbsolutePosition = Rotation2d.kZero;

        public double[] odometryDrivePositionsRad = new double[0];
        public double[] odometryDriveVelocityRadsPerSec = new double[0];
        public Rotation2d[] odometryAnglePositions = new Rotation2d[0];
    }

    /** Update inputs */
    public void updateInputs(SwerveModuleInputs inputs);

    /** Set voltage on drive motor */
    public void runDriveOpenLoop(double output);

    /** Set voltage on angle motor */
    public void runAngleOpenLoop(double output);

    /** Set velocity setpoint */
    public void runDriveVelocity(double velocityRadPerSec, double feedforward);

    /** Set angle setpoint */
    public void runAnglePosition(Rotation2d rotation);

    /** Set PID constants for drive motor */
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA);

    /** Set PID constants for angle motor */
    public void setAnglePID(double kP, double kI, double kD);

    /** Set drive brake mode */
    public void setDriveBrakeMode(boolean enabled);

    /** Set angle brake mode */
    public void setAngleBrakeMode(boolean enabled);

}
