package frc.robot.subsystems.swerve.mod;

import static edu.wpi.first.units.Units.Meters;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Simulation implementation for Swerve Module */
@NullMarked
public class SwerveModuleSim implements SwerveModuleIO {

    private static final DCMotor driveMotorModel = DCMotor.getKrakenX60(1);
    private static final DCMotor turnMotorModel = DCMotor.getFalcon500(1);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, Constants.Swerve.driveGearRatio),
        driveMotorModel);
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, Constants.Swerve.angleGearRatio),
        turnMotorModel);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(0.1, 0, 0, TimedRobot.kDefaultPeriod);
    private PIDController turnController = new PIDController(10.0, 0, 0, TimedRobot.kDefaultPeriod);
    private double driveFFVolts = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /** Simulation implementation for Swerve Module */
    public SwerveModuleSim() {
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelRadius.in(Meters),
            new Rotation2d(turnSim.getAngularPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveSim.getAngularPositionRad() * Constants.Swerve.wheelRadius.in(Meters),
            new Rotation2d(turnSim.getAngularPosition()));
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(TimedRobot.kDefaultPeriod);
        turnSim.update(TimedRobot.kDefaultPeriod);

        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.angleConnected = true;
        inputs.anglePosition = new Rotation2d(turnSim.getAngularPosition());
        inputs.angleAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
        inputs.angleSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
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
