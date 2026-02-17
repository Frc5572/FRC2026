package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.List;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
        public static final int testControllerId = 0;
        /** Stick axis controls less than this amount are treated as 0. */
        public static final double stickDeadband = 0.1;

        /** Maximum Translational speed (in m/s) */
        public static final double driverTranslationalMaxSpeed = 3.0;
        /** Maximum Rotational speed (in rad/s) */
        public static final double driverRotationalMaxSpeed = 4.0;
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
        public static final double KP = 18;
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
    }

    /**
     * MoveToPos Constants
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
        /** Distance between right and left wheels on robot */
        public static final double trackWidth = Units.inchesToMeters(21.7);
        /** Distance between front and back wheels on robot */
        public static final double wheelBase = Units.inchesToMeters(21.8);
        /** Distance from the center of the wheel to the ground */
        public static final Distance wheelRadius = Inches.of(1.913);
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

        /* Module Gear Ratios */
        /** Swerve Drive Motor Gear Ratio */
        public static final double driveGearRatio = (8.14 / 1.0); // MK4i L1
        /** Swerve Angle Motor Gear Ratio */
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // (150 / 7) : 1

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

        /* Angle Motor PID Values */
        /** Proportional Swerve Angle Motor PID Value */
        public static final double angleKP = 100.0;
        /** Integral Swerve Angle Motor PID Value */
        public static final double angleKI = 0.0;
        /** Derivative Swerve Angle Motor PID Value */
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */
        /** Proportional Swerve Drive Motor PID Value */
        public static final double driveKP = 0.0012;
        /** Integral Swerve Drive Motor PID Value */
        public static final double driveKI = 0.0;
        /** Derivative Swerve Drive Motor PID Value */
        public static final double driveKD = 0.0;
        /** Feedforward Swerve Drive Motor PID Value */
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        /** Static Swerve Drive Motor Characterization Value */
        public static final double driveKS = 0.185;
        /** Velocity Swerve Drive Motor Characterization Value */
        public static final double driveKV = 1.004 / 6.536;
        /** Acceleration Swerve Drive Motor Characterization Value */
        public static final double driveKA = 0.0;

        /* Swerve Profiling Values */
        /** Max Speed in Meters per Second */
        public static final double maxSpeed = 3.0;
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
            new ModuleConstantsBuilder()
                .driveMotorId(1)
                .angleMotorId(0)
                .canCoderId(1)
                .angleOffset(Rotation2d.fromRotations(0.106445))
                .finish(),
            // Front Right Module
            new ModuleConstantsBuilder()
                .driveMotorId(7)
                .angleMotorId(6)
                .canCoderId(2)
                .angleOffset(Rotation2d.fromRotations(0.409668))
                .finish(),
            // Back Left Module
            new ModuleConstantsBuilder()
                .driveMotorId(2)
                .angleMotorId(3)
                .canCoderId(3)
                .angleOffset(Rotation2d.fromRotations(0.474121))
                .finish(),
            // Back Right Module
            new ModuleConstantsBuilder()
                .driveMotorId(5)
                .angleMotorId(4)
                .canCoderId(4)
                .angleOffset(Rotation2d.fromRotations(-0.402344))
                .finish(),
        };
        // @formatter:on
    }

    /** Trench MoveToPose Constants */
    public static final class Trench {

        // translation tolerance value for move to trench
        public static final double tolerance = 0.08;

        // Creates a list of all of the trench locations
        public static final List<Translation2d> trenchLocation =
            List.of(FieldConstants.LeftTrench.redTrenchCenterLeft,
                FieldConstants.LeftTrench.blueTrenchCenterLeft,
                FieldConstants.RightTrench.redTrenchCenterRight,
                FieldConstants.RightTrench.blueTrenchCenterRight);
    }

    /** Vision Constants */
    public static final class Vision {
        public static final AprilTagFieldLayout fieldLayout =
            FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();

        public static final Pose3d turretCenter =
            new Pose3d(new Translation3d(-0.1651, 0, 0.36772), Rotation3d.kZero);

        /** TODO: meausre on bot */
        public static final Pose3d turretLeft =
            new Pose3d(Inches.of(8.240), Inches.of(8.131), Inches.of(0.0), Rotation3d.kZero);
        public static final Pose3d turretRight =
            new Pose3d(Inches.of(5.307), Inches.of(-5.958), Inches.of(0.0), Rotation3d.kZero);

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
            new CameraConstantsBuilder()
                .name("rightTurretCamera")
                .height(800)
                .width(1280)
                .horizontalFieldOfView(80)
                .simFps(20)
                .simLatency(0.3)
                .simLatencyStdDev(0.02)
                .calibrationErrorMean(0.8)
                .calibrationErrorStdDev(0.08)
                .robotToCamera(new Transform3d(turretCenter, turretRight))
                .translationError(0.02)
                .rotationError(0)
                .singleTagError(0)
                .isTurret(true)
                .finish(),
        };
        // @formatter:on
    }

    /** Adjustable Hood Constants */
    public static final class AdjustableHood {
        public static final int HoodMotorID = 11;

        /* PID Values */
        /** Proportional PID Value for hood position control. */
        public static final double KP = 0.0;
        /** Integral PID Value for hood position control. */
        public static final double KI = 0.0;
        /** Derivative PID Value for hood position control. */
        public static final double KD = 0.0;

        /* Characterization Values */
        /** Static Characterization Value for overcoming friction. */
        public static final double KS = 0.0;
        /** Velocity Characterization Value */
        public static final double KV = 0.0;
        /** Acceleration Characterization Value */
        public static final double KA = 0.0;
        /** Gravity Characterization Value */
        public static final double KG = 0.0;

        public static final Angle hoodMaxAngle = Degrees.of(0.0);
        public static final Angle hoodMinAngle = Degrees.of(0.0);

        public static final double MMAcceleration = 0.0;
        public static final double MMCVelocity = 0.0;
        public static final double MMJerk = 0.0;

        public static final double hoodTolerence = 0.05;

        public static final SensorDirectionValue hoodCANCoderInvert =
            SensorDirectionValue.Clockwise_Positive;
        public static final double hoodCANcoderDiscontinuity = 0.5;
    }

    /**
     * Climber subsystem constants.
     *
     * <p>
     * Contains all configuration values for the climber's telescope extension and pivot rotation
     * mechanisms, including motor IDs, PID gains, feedforward constants, motion constraints, and
     * preset positions.
     */
    public static final class Climber {

        /**
         * Telescope extension mechanism constants.
         *
         * <p>
         * Defines motor IDs and neutral mode for the left and right telescope motors.
         */
        public static final class Telescope {

            /** CAN ID for the right telescope motor. */
            public static final int RIGHT_ID = 15;

            /** CAN ID for the left telescope motor. */
            public static final int LEFT_ID = 14;

            /** Neutral mode for telescope motors (brake or coast). */
            public static final NeutralModeValue BREAK = NeutralModeValue.Brake;


            /** Telescope length at the top position, in degrees. */
            public static final Angle DEGREES_AT_TOP = Degrees.of(72.0);

            /** Telescope length at the top position, in Meters. */
            public static final Distance ROTATIONS_AT_TOP = Meters.of(220);

            /**
             * S Sensor to mechanism ratio for converting encoder rotations to mechanism angle.
             */
            public static final double SENSOR_TO_MECHANISM_RATIO =
                ROTATIONS_AT_TOP.in(Meters) / DEGREES_AT_TOP.in(Degrees);

            /** Proportional gain for pivot position control. */
            public static final double KP = 50.0;

            /** Integral gain for pivot position control. */
            public static final double KI = 0.0;

            /** Derivative gain for pivot position control. */
            public static final double KD = 0.0;

            /** Static feedforward constant for overcoming friction. */
            public static final double KS = 0.9;

            /** Velocity feedforward constant. */
            public static final double KV = 0.0;

            /** Acceleration feedforward constant. */
            public static final double KA = 0.0;

            /** Gravity feedforward constant for maintaining pivot angle. */
            public static final double KG = 0.9375;

        }

        /**
         * Pivot rotation mechanism constants.
         *
         * <p>
         * Includes motor configuration, PID and feedforward gains for motion magic control, motion
         * constraints, and preset angle positions.
         */
        public static final class Pivot {

            /** CAN ID for the pivot motor. */
            public static final int ID = 8;

            /** Neutral mode for the pivot motor (brake or coast). */
            public static final NeutralModeValue BREAK = NeutralModeValue.Brake;

            /** Proportional gain for pivot position control. */
            public static final double KP = 50.0;

            /** Integral gain for pivot position control. */
            public static final double KI = 0.0;

            /** Derivative gain for pivot position control. */
            public static final double KD = 0.0;

            /** Static feedforward constant for overcoming friction. */
            public static final double KS = 0.9;

            /** Velocity feedforward constant. */
            public static final double KV = 0.0;

            /** Acceleration feedforward constant. */
            public static final double KA = 0.0;

            /** Gravity feedforward constant for maintaining pivot angle. */
            public static final double KG = 0.9375;

            /** Motion magic cruise velocity, in rotations per second. */
            public static final double C_VELOCITY = 4.0;

            /** Motion magic acceleration, in rotations per second squared. */
            public static final double ACCELERATION = 10.0;

            /** Motion magic jerk limit. */
            public static final double JERK = 6000000.0;

            /** Maximum pivot velocity. */
            public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(0.0);

            /** Gear ratio for the pivot mechanism. */
            public static final double GEAR_RATIO = 20.0 / 1.0;

            /** Pivot angle at the top position, in degrees. */
            public static final Angle DEGREES_AT_TOP = Degrees.of(72.0);

            /** Pivot angle at the top position, in radians. */
            public static final Angle ROTATIONS_AT_TOP = Radians.of(220);

            /** Angle of the climber in starting configuration */
            public static final Angle startingAngle = Degrees.of(68.0);

            /**
             * Sensor to mechanism ratio for converting encoder rotations to mechanism angle.
             */
            public static final double SENSOR_TO_MECHANISM_RATIO =
                ROTATIONS_AT_TOP.in(Rotations) / DEGREES_AT_TOP.in(Degrees);
        }
    }

    /** Turret Constants */
    public static final class Turret {
        public static final double motorGearing = 53.934;
        public static final double gear1Gearing = 35.0 / 75.0;
        public static final Rotation2d gear1Offset = Rotation2d.kZero;
        public static final double gear2Gearing = 36.0 / 75.0;
        public static final Rotation2d gear2Offset = Rotation2d.kZero;
        public static final Angle minAngle = Degrees.of(-360);
        public static final Angle maxAngle = Degrees.of(360);

        public static final int TurretMotorID = 19;
        public static final int TurretCANcoderID1 = 5;
        public static final int TurretCANcoderID2 = 6;

        /* PID Values */
        /** Proportional PID Value for turret position control. */
        public static final double KP = 0.0;
        /** Integral PID Value for turret position control. */
        public static final double KI = 0.0;
        /** Derivative PID Value for turret position control. */
        public static final double KD = 0.0;

        /* Characterization Values */
        /** Static Characterization Value for overcoming friction. */
        public static final double KS = 0.0;
        /** Velocity Characterization Value */
        public static final double KV = 0.0;
        /** Acceleration Characterization Value */
        public static final double KA = 0.0;

        public static final double MMCVelocity = 0.0;
        public static final double MMAcceleration = 0.0;
        public static final double MMJerk = 0.0;

        public static final Angle testAngle = Degrees.of(30);

        public static final double turretTolerence = 0.01;

        public static final SensorDirectionValue canCoder1Invert =
            SensorDirectionValue.Clockwise_Positive;
        public static final SensorDirectionValue canCoder2Invert =
            SensorDirectionValue.Clockwise_Positive;
        public static final double turretCANCoderDiscontinuity = 0.5;
    }

    /** Shooter Constants */
    public static final class Shooter {
        /** ID for Shooter Motor 1 */
        public static final int motor1ID = 10;
        /** ID for Shooter Motor 2 */
        public static final int motor2ID = 12;

        /** Motor Invert for Shooter Motors */
        public static final InvertedValue shooterMotorInvert = InvertedValue.Clockwise_Positive;
        /** Motor Alignment for Shooter Motors */
        public static final MotorAlignmentValue shooterMotorAlignment = MotorAlignmentValue.Opposed;
        /** Neutral Mode for Shooter Motors */
        public static final NeutralModeValue shooterNeutralMode = NeutralModeValue.Brake;

        public static final double shooterKS = 0.1;
        public static final double shooterKV = 0.12;
        public static final double shooterKP = 0.11;
        public static final double shooterKI = 0.0;
        public static final double shooterKD = 0.0;
    }
}
