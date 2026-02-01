package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.sim.SimPosition;

/**
 * Shooter Sim Implementation
 */
public class ShooterSim implements ShooterIO {

    private final SimPosition flywheel = new SimPosition(0.5, 2.0, 400.0);
    private double flywheelTarget = 0.0;

    @Override
    public void updateInputs(ShooterInputs inputs) {
        flywheel.update(flywheelTarget);
        inputs.shooterAngularVelocity1 = RadiansPerSecond.of(flywheel.position);
        inputs.shooterAngularVelocity2 = RadiansPerSecond.of(flywheel.position);
    }

    @Override
    public void runShooterVelocity(double velocity) {
        flywheelTarget = velocity;
    }

    public void shootOne() {
        flywheel.position *= 0.6;
        flywheel.velocity = 0.0;
    }

}
