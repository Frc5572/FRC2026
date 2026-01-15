package frc.robot.subsystems.swerve.gyro;

import frc.robot.subsystems.swerve.SwerveSim;

public class GyroSim implements GyroIO {

    private final SwerveSim sim;

    public GyroSim(SwerveSim sim) {
        this.sim = sim;
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = true;
        inputs.yaw = sim.getPose().getRotation();
        inputs.yawRads = new double[] {inputs.yaw.getRadians()};
        inputs.yawVelocityRadPerSec = 0.0;
    }

}
