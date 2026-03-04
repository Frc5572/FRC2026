package frc.robot.util.tunable;

import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Per-module constants */
@NullMarked
public class ModuleConstants implements Tunable {

    /** CAN ID for the drive motor */
    public final int driveMotorId;

    /** CAN ID for the angle motor */
    public final int angleMotorId;

    /** CAN ID for the CANCoder */
    public final int canCoderId;

    /** Reported angle when wheel is straight */
    public final Rotation2d angleOffset;

    /** Per-module constants */
    @TypeStateBuilder("ModuleConstantsBuilder")
    public ModuleConstants(@RequiredField int driveMotorId, @RequiredField int angleMotorId,
        @RequiredField int canCoderId, @RequiredField Rotation2d angleOffset) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId = canCoderId;
        this.angleOffset = angleOffset;
    }

}
