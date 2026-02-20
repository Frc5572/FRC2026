package frc.robot.subsystems.swerve.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/**
 * Control scheme utilities for teleoperated swerve driving.
 *
 * <p>
 * This class converts raw driver input signals (typically joystick axes) into {@link ChassisSpeeds}
 * suitable for commanding a swerve drivetrain. Input processing includes deadbanding, non-linear
 * shaping, and scaling to configured maximum translational and rotational speeds.
 *
 * <p>
 * The resulting chassis speeds are intended to be consumed by higher-level drive commands
 * (robot-relative or field-relative) rather than applied directly to hardware.
 */
@NullMarked
public class TeleopControls {

    /**
     * Creates a supplier that converts driver inputs into desired chassis speeds.
     *
     * <p>
     * The returned {@link Supplier} performs the following processing each time it is evaluated:
     * <ul>
     * <li>Applies a configurable deadband to all input axes</li>
     * <li>Applies non-linear shaping to translational inputs for finer low-speed control</li>
     * <li>Scales translational and rotational inputs to configured maximum speeds</li>
     * </ul>
     *
     * <p>
     * The produced {@link ChassisSpeeds} are expressed in a <em>pseudo-field-relative</em> frame,
     * where:
     * <ul>
     * <li>Forward input corresponds to positive X velocity</li>
     * <li>Rightward input corresponds to positive Y velocity</li>
     * <li>Counterclockwise input corresponds to positive angular velocity</li>
     * </ul>
     *
     * <p>
     * This method does not perform true field-relative conversion; callers are expected to apply
     * any required frame transformations using the robot's current heading.
     *
     * @param forward supplier providing the forward/backward driver input
     * @param right supplier providing the left/right driver input
     * @param turnCCW supplier providing the counterclockwise rotation input
     * @return a supplier that generates processed {@link ChassisSpeeds} for teleop control
     */
    public static Supplier<ChassisSpeeds> teleopControls(DoubleSupplier forward,
        DoubleSupplier right, DoubleSupplier turnCCW) {
        return () -> {
            double xaxis = right.getAsDouble();
            double yaxis = forward.getAsDouble();
            double raxis = turnCCW.getAsDouble();
            yaxis = MathUtil.applyDeadband(yaxis, Constants.DriverControls.stickDeadband);
            xaxis = MathUtil.applyDeadband(xaxis, Constants.DriverControls.stickDeadband);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            raxis = (Math.abs(raxis) < Constants.DriverControls.stickDeadband) ? 0 : raxis;
            return new ChassisSpeeds(yaxis * Constants.DriverControls.driverTranslationalMaxSpeed,
                xaxis * Constants.DriverControls.driverTranslationalMaxSpeed,
                raxis * Constants.DriverControls.driverRotationalMaxSpeed);
        };
    }

    public static Supplier<ChassisSpeeds> teleopTranlsationControls(DoubleSupplier forward,
        DoubleSupplier right) {
        return () -> {
            double xaxis = right.getAsDouble();
            double yaxis = forward.getAsDouble();
            yaxis = MathUtil.applyDeadband(yaxis, Constants.DriverControls.stickDeadband);
            xaxis = MathUtil.applyDeadband(xaxis, Constants.DriverControls.stickDeadband);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            return new ChassisSpeeds(yaxis * Constants.DriverControls.driverTranslationalMaxSpeed,
                xaxis * Constants.DriverControls.driverTranslationalMaxSpeed, 0.0);
        };
    }

}
