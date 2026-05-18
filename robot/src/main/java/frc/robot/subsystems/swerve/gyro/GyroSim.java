package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.jspecify.annotations.NullMarked;

/** Simulation implementation for gyro */
@NullMarked
public class GyroSim implements GyroIO {

    private final GyroSimulation gyro;

    /** Simulation implementation for gyro */
    public GyroSim(GyroSimulation gyro) {
        this.gyro = gyro;
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.yaw = gyro.getGyroReading();
        inputs.yawVelocityRadPerSec = gyro.getMeasuredAngularVelocity().in(RadiansPerSecond);
        inputs.yawRads = new double[] {inputs.yaw.getRadians()};
        inputs.connected = true;
    }

}
