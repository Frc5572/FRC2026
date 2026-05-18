package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.swerve.mod.ModuleConstants;
import frc.robot.subsystems.swerve.mod.ModuleConstants.ModuleKind;
import frc.robot.subsystems.swerve.mod.ModuleConstantsBuilder;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.CameraConstantsBuilder;
import frc.robot.util.PIDConstants;
import frc.robot.util.PIDConstantsBuilder;

/**
 * Constants file.
 */
public final class Constants {

    public static final double visionFudgeFactor = 0.0;

    public static final boolean tunable = true;

    public static final boolean keepInField = false;

    /** Constants for driver controls */
    public static class DriverControls {
        /** Stick axis controls less than this amount are treated as 0. */
        public static final double stickDeadband = 0.1;

        /** Maximum Translational speed (in m/s) */
        public static final double driverTranslationalMaxSpeed = 4.0;
        /** Maximum Rotational speed (in rad/s) */
        public static final double driverRotationalMaxSpeed = 4.0;

        /** Maximum Translational speed while shooting (in m/s) */
        public static final double driverTranslationalShootSpeed = 1.0;
        /** Maximum Rotational speed while shooting (in rad/s) */
        public static final double driverRotationalShootSpeed = 1.5;
    }

    /**
     * Intake Constants
     */
    public static class IntakeConstants { // change all variables before testing
        public static final int hopperLeftID = 16;
        public static final int hopperRightID = 17;
        public static final int intakeID = 13;
        public static final double hopperMaxDistance = 0;
        public static final double intakeSpeed = 0;
        public static final double hopperMinDistance = 0;
        public static final Distance hopperOutDistance = Meters.of(0.283027);
        public static final double hopperTuckedDistance = 0;
        public static final double KP = 6;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final int limitSwitchID = 9;
        public static final double pinionDiameter = Units.inchesToMeters(2);
        public static final double gearRatio = 21.0 / 20.0;
    }

    /**
     * Indexer Constants
     */
    public static class Indexer {
        public static final int indexerID = 18;
        public static final int spinMotorID = 9;
        public static final int indexerSpeed = 0;
        public static final int spinMotorSpeed = 0;

        public static final double timeReversingDuringUnjam = 0.25;
        public static final double timeBetweenUnjams = 1.0;
        public static final double shooterNotShootingUnjamThreshold = 1.0;
    }

    /**
     * MoveToPos Constants
     */
    public static class SwerveTransformPID {
        public static final double translationP = 2.5;
        public static final double translationI = 0.0;
        public static final double translationD = 0.0;
        public static final double rotationP = 2.0;
        public static final double rotationI = 0.0;
        public static final double rotationD = 0.0;

        public static final double maxAngularVelocity = 6.0;
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
        /** Distance between right and left wheels on robot */
        public static final double trackWidth = Units.inchesToMeters(21.7);
        /** Distance between front and back wheels on robot */
        public static final double wheelBase = Units.inchesToMeters(21.8);
        /** Distance from the center of the wheel to the ground */
        public static final Distance wheelRadius = Inches.of(1.906);
        /** Diameter of the wheels, twice the radius */
        public static final Distance wheelDiameter = wheelRadius.times(2);
        /** Circumference of the wheels */
        public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

        /** Bumper to bumper length of the robot */
        public static final Distance bumperFront = Inches.of(17.5);
        /** Bumper to bumper back of the robot */
        public static final Distance bumperRight = Inches.of(17.5);

        public static final Translation2d[] swerveTranslations =
            new Translation2d[] {new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)};

        /**
         * Swerve Drive Kinematics Object for Rectangular/square 4 Module Swerve
         *
         * <p>
         * No need to ever change this unless you are not doing a traditional rectangular/square 4
         * module swerve.
         */
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(swerveTranslations);

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;

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

        // @formatter:off
        public static final PIDConstants angleMotorPID =
            new PIDConstantsBuilder("Swerve/angle", GravityTypeValue.Elevator_Static)
                .kP(100.0)
                .kI(0.0)
                .kD(0.0)
                .kV(0.0)
                .kS(0.0)
                .kG(0.0)
                .kA(0.0)
                .finish();
        // @formatter:on

        // @formatter:off
        public static final PIDConstants driveMotorPID =
            new PIDConstantsBuilder("Swerve/drive", GravityTypeValue.Elevator_Static)
                .kP(0.0012)
                .kI(0.0)
                .kD(0.0)
                .kV(0.990 / 6.536)
                .kS(0.251)
                .kG(0.0)
                .kA(0.0)
                .finish();
        // @formatter:on

        /* Swerve Profiling Values */
        /** Max Speed in Meters per Second */
        public static final double maxSpeed = 7.0;
        /** Max Speed for Auto in Meters per Second */
        public static final double autoMaxSpeed = 3.0;
        /** Max Angular Velocity in Radians per Second */
        public static final double maxAngularVelocity = 4.0;

        /* Neutral Modes */
        /** Angle Motor Neutral Mode */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        /** Drive Motor Neutral Mode */
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double odometryFrequency = 100.0;

        /* Teleop limits */
        public static final double forwardLimit = 10.0;
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
            new ModuleConstantsBuilder(ModuleKind.Mk4i)
                .driveMotorId(1)
                .angleMotorId(0)
                .canCoderId(1)
                .angleOffset(Rotation2d.fromRotations(0.106445))
                .finish(),
            // Front Right Module
            new ModuleConstantsBuilder(ModuleKind.Mk4i)
                .driveMotorId(7)
                .angleMotorId(6)
                .canCoderId(2)
                .angleOffset(Rotation2d.fromRotations(0.409668))
                .finish(),
            // Back Left Module
            new ModuleConstantsBuilder(ModuleKind.Mk4n)
                .driveMotorId(2)
                .angleMotorId(3)
                .canCoderId(3)
                .angleOffset(Rotation2d.fromRotations(0.474121))
                .finish(),
            // Back Right Module
            new ModuleConstantsBuilder(ModuleKind.Mk4n)
                .driveMotorId(5)
                .angleMotorId(4)
                .canCoderId(4)
                .angleOffset(Rotation2d.fromRotations(-0.402344))
                .finish(),
        };
        // @formatter:on

        public static final HolonomicDriveController holonomicDriveController =
            new HolonomicDriveController(
                new PIDController(Constants.SwerveTransformPID.translationP,
                    Constants.SwerveTransformPID.translationI,
                    Constants.SwerveTransformPID.translationD),
                new PIDController(Constants.SwerveTransformPID.translationP,
                    Constants.SwerveTransformPID.translationI,
                    Constants.SwerveTransformPID.translationD),
                new ProfiledPIDController(Constants.SwerveTransformPID.rotationP,
                    Constants.SwerveTransformPID.rotationI, Constants.SwerveTransformPID.rotationD,
                    new Constraints(Constants.SwerveTransformPID.maxAngularVelocity,
                        Constants.SwerveTransformPID.maxAngularAcceleration)));
    }

    /** Vision Constants */
    public static final class Vision {

        public static final Pose3d turretCenter =
            new Pose3d(new Translation3d(-0.155575, -0.13335, 0), Rotation3d.kZero);

        public static final Pose3d turretRight = new Pose3d(-0.18097, -0.27012, 0.51406,
            new Rotation3d(0, Units.degreesToRadians(-22.115), 0.0))
                .rotateAround(turretCenter.getTranslation(), new Rotation3d(Rotation2d.kZero));

        // @formatter:off
        public static final CameraConstants TURRET_CAMERA =
            new CameraConstantsBuilder()
                .coProcessorName("ubuntu")
                .name("turret")
                .height(800)
                .width(1280)
                .horizontalFieldOfView(80)
                .simFps(20)
                .simLatency(0.8)
                .simLatencyStdDev(0.02)
                .calibrationErrorMean(0.8)
                .calibrationErrorStdDev(0.08)
                .robotToCamera(new Transform3d(turretCenter, turretRight))
                .translationError(Units.inchesToMeters(6))
                .rotationError(0.3)
                .singleTagError(0)
                .finish();
        // @formatter:on
    }
}
