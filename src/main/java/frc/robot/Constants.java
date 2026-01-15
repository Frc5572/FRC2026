package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.swerve.mod.ModuleConstants;
import frc.robot.subsystems.swerve.mod.ModuleConstantsBuilder;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.CameraConstantsBuilder;

/**
 * Constants file.
 */
public final class Constants {

    /** Constants for driver controls */
    public static class DriverControls {
        /** Driverstation controller Index */
        public static final int controllerId = 0;
        /** Stick axis controls less than this amount are treated as 0. */
        public static final double stickDeadband = 0.1;

        /** Maximum Translational speed (in m/s) */
        public static final double driverTranslationalMaxSpeed = 3.0;
        /** Maximum Rotational speed (in rad/s) */
        public static final double driverRotationalMaxSpeed = 4.0;
    }

    /**
     * MoveToPos constants.
     */
    public static class SwerveTransformPID {
        public static final double translationP = 3.5;
        public static final double translationI = 0.0;
        public static final double translationD = 0.0;
        public static final double rotationP = 3.0;
        public static final double rotationI = 0.0;
        public static final double rotationD = 0.0;

        public static final double maxAngularVelocity = 9.0;
        public static final double maxAngularAcceleration = 9 * 5;
    }

    /**
     * Swerve Constants
     */
    public static final class Swerve {
        /** If true, motors and absolute encoders are on canivore loop. Otherwise on rio. */
        public static final boolean isCanviore = true;

        public static final NavXComType navXID = NavXComType.kMXP_SPI;
        public static final boolean invertGyro = true;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.229);
        public static final double wheelBase = Units.inchesToMeters(24.229);
        public static final Distance wheelRadius = Inches.of(1.913);
        public static final Distance wheelDiameter = wheelRadius.times(2);
        public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

        public static final Distance bumperFront = Inches.of(17.5);
        public static final Distance bumperRight = Inches.of(17.5);

        public static final Translation2d[] swerveTranslations =
            new Translation2d[] {new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)};

        /*
         * Swerve Kinematics No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(swerveTranslations);

        /* Module Gear Ratios */
        public static final double driveGearRatio = (8.14 / 1.0); // MK4i L1
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // (150 / 7) : 1

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert =
            InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentLowerLimit = 40;
        public static final double angleCurrentLowerTimeThreshold = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentLowerLimit = 60;
        public static final double driveCurrentLowerTimeThreshold = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 100.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0012;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.185;
        public static final double driveKV = 1.004 / 6.536;
        public static final double driveKA = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.0;
        public static final double autoMaxSpeed = 3.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = 4.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double odometryFrequency = 100.0;

        /* Teleop limits */
        public static final double forwardLimit = 1000.0;
        public static final double forwardTiltLimit = 1000.0;
        public static final double leftTiltLimit = 1000.0;
        public static final double rightTiltLimit = 1000.0;
        public static final double backTiltLimit = 1000.0;
        public static final double skidLimit = 1000.0;

        /* Module Specific Constants */

        // @anchor:moduleConstants
        // @formatter:off
        public static final ModuleConstants[] modulesConstants = new ModuleConstants[] {
            // Front Left Module
            new ModuleConstantsBuilder()
                .driveMotorId(2)
                .angleMotorId(1)
                .canCoderId(1)
                .angleOffset(Rotation2d.fromRotations(0.008789))
                .finish(),
            // Front Right Module
            new ModuleConstantsBuilder()
                .driveMotorId(9)
                .angleMotorId(8)
                .canCoderId(2)
                .angleOffset(Rotation2d.fromRotations(-0.301758))
                .finish(),
            // Back Left Module
            new ModuleConstantsBuilder()
                .driveMotorId(0)
                .angleMotorId(19)
                .canCoderId(4)
                .angleOffset(Rotation2d.fromRotations(-0.451172))
                .finish(),
            // Back Right Module
            new ModuleConstantsBuilder()
                .driveMotorId(11)
                .angleMotorId(10)
                .canCoderId(3)
                .angleOffset(Rotation2d.fromRotations(0.321777))
                .finish(),
        };
        // @formatter:on
    }

    /** Vision Constants */
    public static final class Vision {
        public static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // @formatter:off
        public static final CameraConstants[] cameraConstants = new CameraConstants[] {
            new CameraConstantsBuilder()
                .name("cam0")
                .height(800)
                .width(1280)
                .horizontalFieldOfView(80)
                .simFps(20)
                .simLatency(0.3)
                .simLatencyStdDev(0.02)
                .calibrationErrorMean(0.8)
                .calibrationErrorStdDev(0.08)
                .robotToCamera(new Transform3d(new Translation3d(Units.inchesToMeters(11),
                    -Units.inchesToMeters(12), Units.inchesToMeters(10)),
                    new Rotation3d(Math.PI, 0, 0)))
                .translationError(0.02)
                .finish(),
        };
        // @formatter:on
    }
}
