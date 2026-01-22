package frc.robot.subsystems.swerve.mod;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

/** Simulation implementation for Swerve Module */
@NullMarked
public class SwerveModuleSim implements SwerveModuleIO {

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private final int id;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController;
    private final PIDController turnController;
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private double kV = 1.0;
    private double targetVelocity = 0.0;

    /** Simulation implementation for Swerve Module */
    public SwerveModuleSim(int index, SwerveModuleSimulation modSim) {
        this.id = index;
        this.moduleSimulation = modSim;
        this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(Constants.Swerve.driveCurrentLimit));
        this.turnMotor =
            moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

        this.driveController = new PIDController(0.5, 0.0, 0.0);
        this.turnController = new PIDController(8.0, 0.0, 0.0);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        if (driveClosedLoop) {
            if (Math.abs(driveController.getSetpoint()) < 0.01) {
                driveAppliedVolts = 0.0;
                driveController.reset();
            } else {
                driveAppliedVolts = driveFFVolts
                    + driveController.calculate(
                        moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond))
                    + kV * targetVelocity;
            }
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts =
                turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        Logger.recordOutput("driveVoltage" + id, driveAppliedVolts);
        Logger.recordOutput("angleVoltage" + id, turnAppliedVolts);

        driveMotor.requestVoltage(Units.Volts.of(driveAppliedVolts));
        turnMotor.requestVoltage(Units.Volts.of(turnAppliedVolts));

        inputs.driveConnected = true;
        inputs.angleConnected = true;
        inputs.absoluteAngleConnected = true;

        inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec =
            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.angleAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.anglePosition = inputs.angleAbsolutePosition;
        inputs.angleVelocityRadPerSec =
            moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);

        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
        inputs.odometryDriveVelocityRadsPerSec = new double[] {inputs.driveVelocityRadPerSec};
        inputs.odometryAnglePositions = new Rotation2d[] {inputs.anglePosition};
    }

    @Override
    public void runDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void runAngleOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveClosedLoop = true;
        driveController.setSetpoint(velocityRadPerSec);
        targetVelocity = velocityRadPerSec;
    }

    @Override
    public void runAnglePosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        // Changing PID not handled in sim
    }

    @Override
    public void setAnglePID(double kP, double kI, double kD) {
        // Changing PID not handled in sim
    }

    @Override
    public void setDriveBrakeMode(boolean enabled) {
        // Brake mode not handled in sim
    }

    @Override
    public void setAngleBrakeMode(boolean enabled) {
        // Brake mode not handled in sim
    }

}
