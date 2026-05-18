package frc.robot.subsystems.swerve.util;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Collection of command factories used for drivetrain tuning and characterization.
 *
 * <p>
 * This class provides reusable command sequences for empirically determining drivetrain parameters
 * such as feedforward gains and effective wheel radius. These commands are intended to be run
 * manually during calibration and development, not during normal match operation.
 *
 * <p>
 * This class is non-instantiable and only contains static factory methods.
 */
public final class TuningCommands {

    private TuningCommands() {}

    private static final double ffStartDelay = 2.0;
    private static final double ffRampRate = 0.1;
    private static final double wheelRadiusMaxVelocity = 0.25;
    private static final double wheelRadiusRampRate = 0.05;

    /**
     * Creates a command to characterize drivetrain feedforward constants.
     *
     * <p>
     * This routine performs a quasi-static ramp test by slowly increasing applied voltage while
     * measuring the resulting drivetrain velocity. The collected data is fit using linear
     * regression to estimate:
     *
     * <ul>
     * <li><b>kS</b> - static friction voltage</li>
     * <li><b>kV</b> - velocity-proportional voltage</li>
     * </ul>
     *
     * <p>
     * The characterization process consists of:
     * <ol>
     * <li>Orient all modules to face forward</li>
     * <li>Holding zero output to allow modules to fully orient</li>
     * <li>Linearly ramping voltage at a fixed rate</li>
     * <li>Recording velocity and voltage samples each loop</li>
     * </ol>
     *
     * <p>
     * When the command is cancelled, the collected samples are fit and the resulting constants are
     * printed to the console and logged.
     *
     * <p>
     * This routine should be run on flat carpet with minimal disturbances.
     *
     * @param swerve the swerve subsystem being characterized
     * @param runCharacterization consumer that applies a raw voltage command to the drivetrain
     * @param getFFCharacterizationVelocity supplier that returns the current drivetrain velocity
     *        for sampling
     * @return a command that performs feedforward characterization
     */
    public static Command feedforwardCharacterization(Swerve swerve,
        DoubleConsumer runCharacterization, DoubleSupplier getFFCharacterizationVelocity) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),

            // Allow modules to orient
            Commands.run(() -> runCharacterization.accept(0.0), swerve).withTimeout(ffStartDelay),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = timer.get() * ffRampRate;
                runCharacterization.accept(voltage);
                velocitySamples.add(getFFCharacterizationVelocity.getAsDouble());
                voltageSamples.add(voltage);
            }, swerve)

                // When cancelled, calculate and print results
                .finallyDo(() -> {
                    int n = velocitySamples.size();
                    double sumX = 0.0;
                    double sumY = 0.0;
                    double sumXY = 0.0;
                    double sumX2 = 0.0;
                    for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                    }
                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println("********** Drive FF Characterization Results **********");
                    System.out.println("\tkS: " + formatter.format(kS));
                    System.out.println("\tkV: " + formatter.format(kV));
                    Logger.recordOutput("Sysid/FF/kS", kS);
                    Logger.recordOutput("Sysid/FF/kV", kV);
                }));
    }

    /**
     * Creates a command to characterize the effective wheel radius of the drivetrain.
     *
     * <p>
     * This routine estimates wheel radius by rotating the robot in place and comparing the
     * integrated gyro rotation against the average wheel travel. The wheel radius is computed using
     * the relationship:
     *
     * <pre>
     * wheelRadius = (robotRotation * driveBaseRadius) / wheelTravel
     * </pre>
     *
     * <p>
     * The command runs two sequences in parallel:
     * <ul>
     * <li>A drive sequence that smoothly accelerates rotational speed</li>
     * <li>A measurement sequence that integrates gyro rotation and wheel deltas</li>
     * </ul>
     *
     * <p>
     * Intermediate values and the computed wheel radius are logged continuously for validation and
     * offline analysis.
     *
     * <p>
     * This routine assumes:
     * <ul>
     * <li>Accurate gyro measurements</li>
     * <li>Consistent wheel traction during rotation</li>
     * <li>Correct drivetrain geometry constants</li>
     * </ul>
     *
     * @param swerve the swerve subsystem being characterized
     * @param setModuleStates consumer used to command rotational chassis speeds
     * @param getWheelRadiusCharacterizationPositions supplier of wheel position measurements
     * @param gyroYaw supplier of the current robot yaw angle
     * @return a command that performs wheel radius characterization
     */
    public static Command wheelRadiusCharacterization(Swerve swerve,
        Consumer<ChassisSpeeds> setModuleStates,
        Supplier<double[]> getWheelRadiusCharacterizationPositions, Supplier<Rotation2d> gyroYaw) {
        SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        final double driveBaseRadius =
            Math.hypot(Constants.Swerve.trackWidth / 2.0, Constants.Swerve.wheelBase / 2.0);

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                Commands.run(() -> {
                    double speed = limiter.calculate(wheelRadiusMaxVelocity);
                    setModuleStates.accept(new ChassisSpeeds(0.0, 0.0, speed));
                }, swerve)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = getWheelRadiusCharacterizationPositions.get();
                    state.lastAngle = gyroYaw.get();
                    state.gyroDelta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                    var rotation = gyroYaw.get();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;

                    double[] positions = getWheelRadiusCharacterizationPositions.get();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

                    Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                    Logger.recordOutput("Drive/WheelRadius", Units.metersToInches(wheelRadius));
                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}
