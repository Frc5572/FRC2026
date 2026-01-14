package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO
public interface TurretIO {

    @AutoLog
    public static class TurretInputs {
        public Rotation2d gear1AbsoluteAngle;
        public Rotation2d gear2AbsoluteAngle;
    }

    public void updateInputs(TurretInputs inputs);

    public void setTargetAngle(Rotation2d angle);

}
