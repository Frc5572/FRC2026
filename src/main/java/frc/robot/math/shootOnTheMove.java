package frc.robot.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.shotdata.ShotData;
import frc.robot.shotdata.ShotData.ShotParams;

/** Shoot while moving */
public class ShootOnTheMove {

    record MovingShot(Rotation2d turretAngleRobotRelative, Angle pitch, LinearVelocity exitSpeed,
        boolean feasible) {
    }

    static final Translation2d SHOOTER_OFFSET = new Translation2d(-0.155575, -0.13335);
    static final double LATENCY_SEC = 0.0; // tune: vision + feed + spin-up latency

    static final Angle MIN_PITCH = Degrees.of(90 - 13 - Constants.AdjustableHood.maxHoodAngleDeg);
    static final Angle MAX_PITCH = Degrees.of(90 - 13);
    static final LinearVelocity MAX_EXIT_SPEED =
        MetersPerSecond.of(Constants.Shooter.atSpeedThreshold);

    MovingShot solveMovingShot(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation2d hub,
        double flywheelSpeed) {
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        double currentFlywheelSpeed = flywheelSpeed;

        // 1. Latency compensation: predict where the robot is when the ball leaves
        Pose2d pose = new Pose2d(
            robotPose.getTranslation().plus(new Translation2d(vx, vy).times(LATENCY_SEC)),
            robotPose.getRotation().plus(new Rotation2d(omega * LATENCY_SEC)));

        // 2. Shooter position and velocity in the field frame
        // v_shooter = v_center + omega x r
        Translation2d offsetField = SHOOTER_OFFSET.rotateBy(pose.getRotation());
        Translation2d shooterPos = pose.getTranslation().plus(offsetField);
        double svx = vx - omega * offsetField.getY();
        double svy = vy + omega * offsetField.getX();

        // 3. Radial/tangential frame about the hub
        Translation2d toHub = hub.minus(shooterPos);
        double d = toHub.getNorm();
        Rotation2d rHat = toHub.getAngle();
        double cosR = rHat.getCos();
        double sinR = rHat.getSin();

        double vrRobot = svx * cosR + svy * sinR; // + toward hub
        double vtRobot = -svx * sinR + svy * cosR; // + is CCW (r-hat rotated +90°)
        double vzRobot = 0.0; // assume flat field

        // 4. Stationary solution -> required field-relative ball velocity
        ShotParams s = ShotData.staticShotParameters(d, currentFlywheelSpeed);
        double stationaryPitch = s.pitch().in(Radians);
        double stationarySpeed = s.exitSpeed().in(MetersPerSecond);
        double vrTarget = stationarySpeed * Math.cos(stationaryPitch);
        double vzTarget = stationarySpeed * Math.sin(stationaryPitch);

        // 5. Robot-relative launch vector = target field velocity - shooter velocity
        double a = vrTarget - vrRobot; // radial
        double b = -vtRobot; // tangential (target v_t is 0)
        double c = vzTarget - vzRobot; // vertical

        // 6. Convert to spherical coordinates
        double h = Math.hypot(a, b); // horizontal launch speed, >= 0
        double deltaYaw = Math.atan2(b, a);
        Angle pitch = Radians.of(Math.atan2(c, h));
        LinearVelocity exitSpeed = MetersPerSecond.of(Math.sqrt(h * h + c * c));

        // 7. Turret angle: field heading, then robot-relative
        Rotation2d turretField = rHat.plus(new Rotation2d(deltaYaw));
        Rotation2d turretRobot = turretField.minus(pose.getRotation());

        // 8. Feasibility against mechanism limits
        boolean feasible =
            pitch.gte(MIN_PITCH) && pitch.lte(MAX_PITCH) && exitSpeed.lte(MAX_EXIT_SPEED);

        return new MovingShot(turretRobot, pitch, exitSpeed, feasible);
    }
}
