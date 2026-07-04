package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Random;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.tunable.PIDConstants;

/**
 * Simulation implementation of {@link TurretIO}.
 *
 * <p>
 * Models the turret as a real {@link DCMotorSim} (Kraken through the {@code motorToTurretGearing}
 * reduction) driven by the same Phoenix {@code PositionVoltage} controller used on the robot
 * (kS + kV*velocityFF + kP*positionError, clamped to +/-12 V). Unlike the previous instantaneous
 * model this reproduces the finite slew the real turret shows on large swings. The single tunable
 * is the turret moment of inertia {@link #TURRET_MOI}, fit against {@code /Turret/RelativeAngle} via
 * the kReplayCompare loop.
 * </p>
 */
public class TurretSim implements TurretIO {

    /** Turret moment of inertia, kg*m^2 (about the rotation axis). Larger = slower swings. */
    private static final double TURRET_MOI = 0.03;
    private static final double GEARING = Constants.Turret.motorToTurretGearing;
    private static final double START = 0.0;

    private final Random random;
    private final DCMotorSim turretSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), TURRET_MOI, GEARING),
        DCMotor.getKrakenX60(1));

    /** Commanded target angle, radians (robot-relative). Consumed by the vision/shot/viz models. */
    public double turretTarget = START;
    private double targetVelRotPerSec = 0.0;

    // Position controller gains (turret-rotation based), defaulted to Constants.Turret.pid.
    private double kP = Constants.Turret.pid.kP;
    private double kV = Constants.Turret.pid.kV;
    private double kS = Constants.Turret.pid.kS;

    public TurretSim(Random random) {
        this.random = random;
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        double measRot = turretSim.getAngularPositionRotations();
        double targetRot = Units.radiansToRotations(turretTarget - START);
        double err = targetRot - measRot;
        double volts = kS * Math.signum(err) + kV * targetVelRotPerSec + kP * err;
        volts = MathUtil.clamp(volts, -12.0, 12.0);
        turretSim.setInputVoltage(volts);
        turretSim.update(TimedRobot.kDefaultPeriod);

        double noisyRot = turretSim.getAngularPositionRotations() + 0.0005 * (random.nextDouble() - 0.5);
        inputs.relativeAngle = noisyRot;
        inputs.velocity = turretSim.getAngularVelocity();
        // SimCompare: predicted (actual simulated) angle in rotations to match /Turret/RelativeAngle,
        // plus velocity (rad/s) and the commanded target, for kReplayCompare overlay.
        Logger.recordOutput("SimCompare/Turret/predictedRotations", noisyRot);
        Logger.recordOutput("SimCompare/Turret/predictedVelocityRadPerSec",
            inputs.velocity.in(RadiansPerSecond));
        Logger.recordOutput("SimCompare/Turret/targetRotations", targetRot);
    }

    @Override
    public void setTurretVoltage(Voltage volts) {
        turretSim.setInputVoltage(MathUtil.clamp(volts.in(edu.wpi.first.units.Units.Volts), -12, 12));
        turretSim.update(TimedRobot.kDefaultPeriod);
    }


    @Override
    public void setTargetAngle(Angle angle, AngularVelocity velocity) {
        turretTarget = angle.in(edu.wpi.first.units.Units.Radians);
        targetVelRotPerSec = velocity.in(RotationsPerSecond);
    }

    @Override
    public void resetPosition(Angle angle) {
        turretSim.setState(angle.in(Rotations) * 2 * Math.PI, 0.0);
    }

    @Override
    public void setPID(PIDConstants constants) {
        this.kP = constants.kP;
        this.kV = constants.kV;
        this.kS = constants.kS;
    }
}
