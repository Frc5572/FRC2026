package frc.robot.util.tunable;

import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.typestate.InitField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Per-module constants */
@NullMarked
public class ModuleConstants implements Tunable {

    /** Constants for vendor swerve modules */
    public static enum ModuleKind {
        Mk4i(8.14, 150.0 / 7.0), Mk4n(7.13, 18.75);

        public final double driveGearRatio;
        public final double angleGearRatio;

        ModuleKind(double driveGearRatio_, double angleGearRatio_) {
            driveGearRatio = driveGearRatio_;
            angleGearRatio = angleGearRatio_;
        }


    }

    public final ModuleKind kind;

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
    public ModuleConstants(@InitField ModuleKind kind, @RequiredField int driveMotorId,
        @RequiredField int angleMotorId, @RequiredField int canCoderId,
        @RequiredField Rotation2d angleOffset) {
        this.kind = kind;
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId = canCoderId;
        this.angleOffset = angleOffset;
    }

}
