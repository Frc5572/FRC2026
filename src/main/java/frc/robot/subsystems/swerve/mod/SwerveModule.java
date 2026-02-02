package frc.robot.subsystems.swerve.mod;

import static edu.wpi.first.units.Units.Meters;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Swerve Module */
@NullMarked
public class SwerveModule {

    private final String inputsName;
    private final SwerveModuleIO io;
    public final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    /**
     * Swerve Module
     *
     * @param moduleId index into the motor constants
     * @param io IO implementation
     */
    public SwerveModule(int moduleId, SwerveModuleIO io) {
        this.inputsName = "Swerve/Module" + moduleId;
        this.io = io;
    }

    /**
     * Update inputs for a Swerve Module.
     */
    public void updateInputs() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs(this.inputsName, this.inputs);
    }

    /**
     * Set the desired state of the Swerve Module
     *
     * @param desiredState The desired {@link SwerveModuleState} for the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getState().angle);
        this.io.runAnglePosition(desiredState.angle);
        this.io.runDriveVelocity(
            desiredState.speedMetersPerSecond / Constants.Swerve.wheelRadius.in(Meters), 0);
    }

    /** Get the current Swerve Module State */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocityRadPerSec * Constants.Swerve.wheelRadius.in(Meters),
            inputs.anglePosition);
    }

    /** Get the current Swerve Module Position */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad * Constants.Swerve.wheelRadius.in(Meters),
            inputs.anglePosition);
    }

    /** Run motor at given voltage with angle motor pointing forward */
    public void runCharacterization(double output) {
        io.runDriveOpenLoop(output);
        io.runAnglePosition(Rotation2d.kZero);
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

    /** Set drive motor brake mode */
    public void setBrakeMode(boolean enabled) {
        this.io.setDriveBrakeMode(enabled);
    }

    /** Set angle motor brake mode */
    public void setFreeSpin(boolean enabled) {
        this.io.setAngleBrakeMode(!enabled);
    }

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[0];

    /** Periodic function (doesn't update inputs) */
    public void periodic() {
        int sampleCount = inputs.odometryDrivePositionsRad.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters =
                inputs.odometryDrivePositionsRad[i] * Constants.Swerve.wheelRadius.in(Meters);
            Rotation2d angle = inputs.odometryAnglePositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /** Get swerve module position at timestamp index i */
    public SwerveModulePosition getOdometryPosition(int i) {
        return odometryPositions[i];
    }

}
