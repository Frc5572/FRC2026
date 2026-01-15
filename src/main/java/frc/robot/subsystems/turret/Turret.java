package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem representing the robot turret.
 */
public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();

    /**
     * Creates a new Turret subsystem.
     *
     * @param io Hardware abstraction used to read sensors and control actuators
     */
    public Turret(TurretIO io) {
        super("Turret");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Angle turretRotationEstimate =
            getTurretAngleFromGears(inputs.gear1AbsoluteAngle, inputs.gear2AbsoluteAngle);
        Logger.recordOutput("Turret/EstimatedTurretAngle", turretRotationEstimate);
        Logger.recordOutput("Turret/EstimatedTurretAngleDeg", turretRotationEstimate.in(Degrees));
    }

    /**
     * Estimates the turret's absolute rotation using two gear-mounted absolute encoders.
     *
     * <p>
     * This method performs a search across the valid turret angle range. For each candidate turret
     * angle, it computes the expected encoder readings and scores how closely they match the
     * measured values. The angle with the lowest error is selected.
     * </p>
     *
     * <p>
     * This approach allows reconstruction of an absolute turret angle even when individual encoders
     * wrap multiple times over the turret's range of motion.
     * </p>
     *
     * @param gear1 Absolute angle of the first gear encoder
     * @param gear2 Absolute angle of the second gear encoder
     * @return Best estimate of the turret's absolute rotation
     */
    private static Angle getTurretAngleFromGears(Rotation2d gear1, Rotation2d gear2) {
        Angle turretAngleLimited = getTurretAngleFromGearLimited(gear1,
            Constants.Turret.gear1Gearing, Constants.Turret.gear1Offset);
        Logger.recordOutput("TurretCalcs/turretAngleLimited", turretAngleLimited.in(Degrees));
        Angle step = Rotations.of(Constants.Turret.gear1Gearing);
        do {
            turretAngleLimited = turretAngleLimited.minus(step);
        } while (turretAngleLimited.gt(Constants.Turret.minAngle));
        turretAngleLimited = turretAngleLimited.plus(step);
        Logger.recordOutput("TurretCalcs/Target/Gear1", gear1.getDegrees());
        Logger.recordOutput("TurretCalcs/Target/Gear2", gear2.getDegrees());
        double minScore = Double.MAX_VALUE;
        Angle currentBest = turretAngleLimited;
        int count = 0;
        while (turretAngleLimited.lt(Constants.Turret.maxAngle)) {
            Rotation2d gear1Guess = getGearAnglesFromTurret(turretAngleLimited,
                Constants.Turret.gear1Gearing, Constants.Turret.gear1Offset);
            Rotation2d gear2Guess = getGearAnglesFromTurret(turretAngleLimited,
                Constants.Turret.gear2Gearing, Constants.Turret.gear2Offset);
            double score = normalize(gear2.minus(gear2Guess)).getRadians();
            score = score * score;
            Logger.recordOutput("TurretCalcs/Step" + count + "/TurretAngle",
                turretAngleLimited.in(Degrees));
            Logger.recordOutput("TurretCalcs/Step" + count + "/Gear1Guess",
                gear1Guess.getDegrees());
            Logger.recordOutput("TurretCalcs/Step" + count + "/Gear2Guess",
                gear2Guess.getDegrees());
            Logger.recordOutput("TurretCalcs/Step" + count + "/score", score);
            if (score < minScore) {
                minScore = score;
                currentBest = turretAngleLimited;
            }
            turretAngleLimited = turretAngleLimited.plus(step);
            count++;
        }
        return currentBest;
    }

    /**
     * Computes the expected gear encoder angle for a given turret rotation.
     *
     * @param rotation Absolute turret rotation
     * @param gearing Gear ratio between turret and encoder
     * @param offset Encoder offset applied during calibration
     * @return Normalized expected encoder angle
     */
    public static Rotation2d getGearAnglesFromTurret(Angle rotation, double gearing,
        Rotation2d offset) {
        return normalize(new Rotation2d(rotation.div(gearing).plus(offset.getMeasure())));
    }

    /**
     * Computes a turret rotation estimate from a single gear encoder, without accounting for angle
     * wrapping.
     *
     * <p>
     * This result is only valid modulo the gear ratio and must be further processed to determine
     * the true absolute turret angle.
     * </p>
     *
     * @param gear Absolute angle of the gear encoder
     * @param gearing Gear ratio between turret and encoder
     * @param offset Encoder offset applied during calibration
     * @return Turret angle estimate prior to wrap disambiguation
     */
    private static Angle getTurretAngleFromGearLimited(Rotation2d gear, double gearing,
        Rotation2d offset) {
        return gear.getMeasure().minus(offset.getMeasure()).times(gearing);
    }

    /**
     * Normalizes a rotation to the range (-π, π].
     *
     * @param rot Rotation to normalize
     * @return Normalized rotation
     */
    private static Rotation2d normalize(Rotation2d rot) {
        return new Rotation2d(rot.getCos(), rot.getSin());
    }

}
