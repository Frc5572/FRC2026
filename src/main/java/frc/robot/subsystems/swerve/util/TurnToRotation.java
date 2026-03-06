package frc.robot.subsystems.swerve.util;

import java.util.function.Supplier;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * This command will turn the robot to a specified angle.
 */
public class TurnToRotation extends Command {
    private Swerve swerve;
    private boolean isRelative;
    private double goal;
    private Supplier<Rotation2d> angleSupplier;
    private HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(0, 0, 0), new PIDController(0, 0, 0),
        new ProfiledPIDController(Constants.SwerveTransformPID.translationP / 2,
            Constants.SwerveTransformPID.translationI, Constants.SwerveTransformPID.translationD,
            new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.maxAngularVelocity,
                Constants.SwerveTransformPID.maxAngularAcceleration)));
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();
    private int finishCounter = 0;
    private AutoRoutine autoRoutine;
    private EventLoop eventLoop;
    private boolean isActive = false;
    private boolean isCompleted = false;

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     */
    public TurnToRotation(Swerve swerve, double angle, boolean isRelative) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.goal = angle;
        this.angleSupplier = null;
        this.isRelative = isRelative;
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(1)));
        this.eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    }

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Supplier that provides the requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     */
    public TurnToRotation(Swerve swerve, Supplier<Rotation2d> angle, boolean isRelative) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.angleSupplier = angle;
        this.isRelative = isRelative;
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(1)));
        this.eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    }

    public TurnToRotation setAutoRoutine(AutoRoutine routine) {
        this.autoRoutine = routine;
        this.eventLoop = autoRoutine.loop();
        return this;
    }

    /**
     * Returns a trigger that is {@code true} while this command is actively running.
     *
     * <p>
     * If associated with an {@link AutoRoutine}, the trigger is also gated by the routine's active
     * state.
     *
     * @return trigger indicating whether the command is currently active
     */
    public Trigger active() {
        if (autoRoutine != null) {
            return new Trigger(eventLoop,
                () -> this.isActive && autoRoutine.active().getAsBoolean());
        }
        return new Trigger(eventLoop, () -> this.isActive);
    }

    /**
     * Returns a trigger that becomes {@code true} once the command has completed successfully.
     *
     * <p>
     * The trigger remains {@code true} until the command is restarted.
     *
     * @return trigger indicating completion of the command
     */
    public Trigger done() {
        return new Trigger(eventLoop, () -> this.isCompleted);
    }

    @Override
    public void initialize() {
        isActive = true;
        isCompleted = false;
        startPos = swerve.state.getGlobalPoseEstimate();

        // Get the angle - either from supplier or stored goal
        if (angleSupplier != null) {
            this.goal = angleSupplier.get().getDegrees();
        }

        if (isRelative) {
            targetPose2d = new Pose2d(startPos.getTranslation(),
                startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            targetPose2d = new Pose2d(targetPose2d.getTranslation(),
                targetPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        }
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.state.getGlobalPoseEstimate();
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
            targetPose2d, 0, targetPose2d.getRotation());
        swerve.state.updateSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupt) {
        isActive = false;
        isCompleted = !interrupt;
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        if (holonomicDriveController.atReference()) {
            finishCounter++;
        } else {
            finishCounter = 0;
        }
        return finishCounter > 2;
    }
}
