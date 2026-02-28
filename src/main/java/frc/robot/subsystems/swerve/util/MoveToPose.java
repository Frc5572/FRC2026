package frc.robot.subsystems.swerve.util;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.typestate.AltMethod;
import frc.robot.util.typestate.InitField;
import frc.robot.util.typestate.OptionalField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/**
 * Command that drives a swerve drivetrain to a specified field-relative pose using a
 * {@link HolonomicDriveController}.
 *
 * <p>
 * This command continuously computes robot-relative chassis speeds that reduce translational and
 * rotational error to a target {@link Pose2d}. The command finishes once the robot is within the
 * specified translation and rotation tolerances.
 *
 * <p>
 * The target pose may be supplied dynamically (e.g., from vision or an autonomous routine). An
 * optional alliance-based pose flip can be applied automatically for red alliance operation.
 *
 * <p>
 * Output chassis speeds are robot-relative and are clamped to a configurable maximum translational
 * speed.
 *
 * <p>
 * This command is typically created via {@code Swerve.moveToPose(...)} and should not be
 * instantiated directly.
 */
@NullMarked
public class MoveToPose extends Command {

    private final AutoRoutine autoRoutine;
    private final EventLoop eventLoop;

    private final Swerve swerve;
    private final Consumer<ChassisSpeeds> robotRelativeConsumer;
    private final Supplier<Pose2d> pose2dSupplier;
    private final DoubleSupplier maxSpeedSupplier;
    private final boolean flipForRed;
    private final double translationTolerance;
    private final double rotationTolerance;

    private boolean isActive = false;
    private boolean isCompleted = false;

    /**
     * Constructs a command that drives the robot to a target pose.
     *
     * <p>
     * <b>Note:</b> This constructor is intended for use by generated builders. Prefer
     * {@code Swerve.moveToPose(...)} when creating this command.
     *
     * @param swerve drivetrain subsystem providing pose estimation
     * @param robotRelativeConsumer consumer that accepts robot-relative chassis speeds
     * @param target supplier providing the desired field-relative target pose
     * @param autoRoutine optional autonomous routine used to scope triggers
     * @param maxSpeed supplier providing the maximum allowed translational speed (m/s)
     * @param flipForRed whether to mirror the target pose for red alliance
     * @param translationTolerance allowable positional error (meters)
     * @param rotationTolerance allowable angular error (radians)
     */
    @TypeStateBuilder("MoveToPoseBuilder")
    public MoveToPose(@InitField Swerve swerve,
        @InitField Consumer<ChassisSpeeds> robotRelativeConsumer,
        @RequiredField(alt = @AltMethod(type = Pose2d.class, parameter_name = "targetConst",
            value = "() -> targetConst")) Supplier<Pose2d> target,
        @OptionalField("null") @Nullable AutoRoutine autoRoutine,
        @OptionalField(value = "() -> frc.robot.Constants.Swerve.autoMaxSpeed",
            alt = @AltMethod(type = double.class, parameter_name = "maxSpeedConst",
                value = "() -> maxSpeedConst")) DoubleSupplier maxSpeed,
        @OptionalField("true") boolean flipForRed,
        @OptionalField("0.5") double translationTolerance,
        @OptionalField("edu.wpi.first.math.util.Units.degreesToRadians(5)") double rotationTolerance) {
        this.autoRoutine = autoRoutine;
        if (autoRoutine == null) {
            this.eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
        } else {
            this.eventLoop = autoRoutine.loop();
        }
        this.swerve = swerve;
        this.robotRelativeConsumer = robotRelativeConsumer;
        this.pose2dSupplier = target;
        this.maxSpeedSupplier = maxSpeed;
        this.flipForRed = flipForRed;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;
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
    }


    private Pose2d target = Pose2d.kZero;

    @Override
    public void execute() {
        target = this.pose2dSupplier.get();
        if (flipForRed) {
            target = AllianceFlipUtil.apply(target);
        }
        ChassisSpeeds ctrlEffort = Constants.Swerve.holonomicDriveController
            .calculate(swerve.state.getGlobalPoseEstimate(), target, 0, target.getRotation());
        double speed = Math.hypot(ctrlEffort.vxMetersPerSecond, ctrlEffort.vyMetersPerSecond);
        double maxSpeed = this.maxSpeedSupplier.getAsDouble();
        if (speed > maxSpeed) {
            double mul = maxSpeed / speed;
            ctrlEffort.vxMetersPerSecond *= mul;
            ctrlEffort.vyMetersPerSecond *= mul;
        }
        this.robotRelativeConsumer.accept(ctrlEffort);
    }

    @Override
    public void end(boolean interrupted) {
        isActive = false;
        isCompleted = !interrupted;
    }

    @Override
    public boolean isFinished() {
        Pose2d poseError = Pose2d.kZero.plus(target.minus(swerve.state.getGlobalPoseEstimate()));
        final var eTranslate = poseError.getTranslation();
        final var eRotate = poseError.getRotation();
        return Math.abs(eTranslate.getX()) < translationTolerance
            && Math.abs(eTranslate.getY()) < translationTolerance
            && Math.abs(eRotate.getDegrees()) < rotationTolerance;
    }

}
