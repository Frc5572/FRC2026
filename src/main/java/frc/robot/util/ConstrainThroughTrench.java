package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

public class ConstrainThroughTrench extends Command {

    private EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    private AutoRoutine autoRoutine;
    private final Swerve swerve;
    private final Supplier<Pose2d> pose2dSupplier;
    private final DoubleSupplier maxSpeedSupplier;
    private Pose2d pose2d;
    private final double tol;
    private final double rotTol;
    /** If this trajectory is currently running */
    private boolean isActive = false;
    /** If the trajectory ran to completion */
    private boolean isCompleted = false;

    public ConstrainThroughTrench(Swerve swerve, Supplier<Pose2d> pose2dSupplier,
        DoubleSupplier maxSpeedSupplier, double tol, double rotTol) {
        this.swerve = swerve;
        this.pose2dSupplier = pose2dSupplier;
        this.maxSpeedSupplier = maxSpeedSupplier;
        this.tol = tol;
        this.rotTol = rotTol;
        addRequirements(swerve);
    }

    public ConstrainThroughTrench(Swerve swerve, Supplier<Pose2d> pose2dSupplier, double tol,
        double rotTol, AutoRoutine autoRoutine) {
        this(swerve, pose2dSupplier, () -> Constants.Swerve.maxSpeed, tol, rotTol);
        this.autoRoutine = autoRoutine;
        this.eventLoop = autoRoutine.loop();
    }

    public Trigger active() {
        return new Trigger(eventLoop, () -> isActive);
    }

    public Trigger done() {
        return new Trigger(eventLoop, () -> isCompleted);
    }

    @Override
    public void initialize() {
        isActive = true;
        isCompleted = false;
        pose2d = pose2dSupplier.get();
    }

    @Override
    public void end(boolean interrupted) {
        isActive = false;
        isCompleted = !interrupted;
    }
}
