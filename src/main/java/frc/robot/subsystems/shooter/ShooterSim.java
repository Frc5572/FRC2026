package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.sim.SimPosition;
import frc.robot.util.tunable.FlywheelConstants;

/**
 * Shooter Sim Implementation
 */
public class ShooterSim implements ShooterIO {

    public final SimPosition flywheel = new SimPosition(20.0, 40.0, 400.0);
    private double flywheelTarget = 0.0;

    @Override
    public void updateInputs(ShooterInputs inputs) {
        flywheel.update(flywheelTarget);
        inputs.shooterAngularVelocity1 = RadiansPerSecond.of(flywheel.position);
        inputs.shooterAngularVelocity2 = RadiansPerSecond.of(flywheel.position);
    }

    @Override
    public void runDutyCycleVelocity(double velocity) {
        flywheelTarget = velocity;
    }

    @Override
    public void runTorqueCurrentVelocity(double velocity) {
        flywheelTarget = velocity;
    }

    public void shootOne() {
        flywheel.position *= 0.9;
        flywheel.velocity *= 0.9;
    }

    @Override
    public void setConstants(FlywheelConstants constants) {

    }

}
