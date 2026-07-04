package frc.robot.subsystems.adjustable_hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Simulation implementation of {@link AdjustableHoodIO}.
 *
 * <p>
 * Models the hood as a real {@link DCMotorSim} (Kraken through the {@code gearRatio} reduction)
 * driven by the same Phoenix {@code PositionVoltage} controller used on the robot (kS + kP*error,
 * clamped to +/-12 V). The stiff kP plus the motor/gearing free speed reproduce the real hood's fast
 * but finite slew (~6.5 rad/s). The single tunable is the hood moment of inertia
 * {@link #HOOD_MOI}, fit against {@code /Adjustable Hood/RelativeAngle} via the kReplayCompare loop.
 * </p>
 */
public class AdjustableHoodSim implements AdjustableHoodIO {

    /** Hood moment of inertia, kg*m^2. Larger = slower slew. */
    private static final double HOOD_MOI = 0.01;
    private static final double GEARING = Constants.AdjustableHood.gearRatio;

    private final DCMotorSim hoodSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), HOOD_MOI, GEARING),
        DCMotor.getKrakenX60(1));

    private final double kP = Constants.AdjustableHood.KP;
    private final double kS = Constants.AdjustableHood.KS;

    private double hoodTargetRot = 0.0;
    private boolean velocityMode = false;
    private double directVolts = 0.0;

    /** Current hood angle in radians, for the shot model. */
    public double getAngleRad() {
        return hoodSim.getAngularPositionRad();
    }

    @Override
    public void updateInputs(AdjustableHoodInputs inputs) {
        double volts;
        if (velocityMode) {
            volts = directVolts;
        } else {
            double err = hoodTargetRot - hoodSim.getAngularPositionRotations();
            volts = kS * Math.signum(err) + kP * err;
        }
        hoodSim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
        hoodSim.update(TimedRobot.kDefaultPeriod);

        inputs.relativeAngle = Radians.of(hoodSim.getAngularPositionRad());
        inputs.velocity = hoodSim.getAngularVelocity();
        // SimCompare: predicted angle in radians to match the real /Adjustable Hood/RelativeAngle,
        // plus the commanded target, for kReplayCompare overlay.
        Logger.recordOutput("SimCompare/Hood/predictedRadians", inputs.relativeAngle.in(Radians));
        Logger.recordOutput("SimCompare/Hood/targetRadians", Rotations.of(hoodTargetRot).in(Radians));
    }

    @Override
    public void setAdjustableHoodVoltage(double volts) {
        velocityMode = true;
        directVolts = volts;
    }

    @Override
    public void setTargetAngle(Angle angle) {
        velocityMode = false;
        hoodTargetRot = angle.in(Rotations);
    }

}
