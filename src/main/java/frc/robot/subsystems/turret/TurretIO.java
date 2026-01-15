package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.GenerateEmptyIO;

/**
 * Hardware abstraction layer for the turret subsystem.
 */
@GenerateEmptyIO
public interface TurretIO {

    /**
     * Container for all turret sensor inputs.
     */
    @AutoLog
    public static class TurretInputs {
        public Rotation2d gear1AbsoluteAngle = Rotation2d.kZero;
        public Rotation2d gear2AbsoluteAngle = Rotation2d.kZero;
        public Angle relativeAngle = Rotations.of(0);
    }

    /**
     * Updates the provided {@link TurretInputs} structure with the latest sensor values.
     */
    public void updateInputs(TurretInputs inputs);

    /**
     * Commands the turret to move toward the specified target angle.
     */
    public void setTargetAngle(Angle angle);

}
