package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroSim;
import frc.robot.subsystems.swerve.mod.SwerveModuleIO;
import frc.robot.subsystems.swerve.mod.SwerveModuleSim;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.tunable.ModuleConstants.ModuleKind;

/** Simulation implementation for swerve */
@NullMarked
public final class SwerveSim implements SwerveIO {

    public final SwerveDriveSimulation mapleSim;

    /** Simulation implementation for swerve */
    public SwerveSim(Pose2d initialPose) {
        this.mapleSim = new SwerveDriveSimulation(
            DriveTrainSimulationConfig.Default().withGyro(COTS.ofNav2X())
                .withRobotMass(Pounds.of(150))
                .withCustomModuleTranslations(Constants.Swerve.swerveTranslations)
                .withBumperSize(Constants.Swerve.bumperFront.times(2),
                    Constants.Swerve.bumperRight.times(2))
                .withSwerveModule(new SwerveModuleSimulationConfig(DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX60(1), ModuleKind.Mk4i.driveGearRatio,
                    ModuleKind.Mk4i.angleGearRatio, Volts.of(0.15), Volts.of(0.35),
                    Constants.Swerve.wheelRadius, KilogramSquareMeters.of(0.02), 1.2)),
            initialPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(this.mapleSim);
    }


    /** Supplier passed into Swerve constructor */
    public SwerveSim simProvider(PhoenixOdometryThread odometryThread) {
        return this;
    }

    /** Supplier passed into Swerve constructor */
    public GyroIO gyroProvider(PhoenixOdometryThread odometryThread) {
        return new GyroSim(this.mapleSim.getGyroSimulation());
    }

    /** Supplier passed into Swerve constructor */
    public SwerveModuleIO moduleProvider(int index, PhoenixOdometryThread odometryThread) {
        return new SwerveModuleSim(index, this.mapleSim.getModules()[index]);
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.timestamps = new double[] {Timer.getTimestamp()};
    }


    @Override
    public void resetPose(Pose2d pose) {
        mapleSim.setSimulationWorldPose(pose);
    }

}
