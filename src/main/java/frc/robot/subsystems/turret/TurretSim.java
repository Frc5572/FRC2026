package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class TurretSim implements TurretIO {

    private Rotation2d turretRotation;

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.gear1AbsoluteAngle = normalize(
            turretRotation.div(Constants.Turret.gear1Gearing).plus(Constants.Turret.gear1Offset));
        inputs.gear2AbsoluteAngle = normalize(
            turretRotation.div(Constants.Turret.gear2Gearing).plus(Constants.Turret.gear2Offset));
    }

    private static Rotation2d normalize(Rotation2d rot) {
        return new Rotation2d(rot.getCos(), rot.getSin());
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        turretRotation = angle;
        Logger.recordOutput("Turret/GTAngle", angle);
    }

}
