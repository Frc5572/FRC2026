package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shotdata.TargetingState;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

/** Command Factory */
public class CommandFactory {

    /** Shoot at a given target. */
    public static Command shoot(TargetingState state, Shooter shooter, Indexer indexer,
        AdjustableHood hood) {
        return Commands.parallel(shooter.shoot(() -> state.getDesiredFlywheelSpeed()),
            hood.setGoal(() -> Degrees.of(state.getDesiredHoodAngleDeg())),
            indexer.runSpindexer(() -> state.isOkayToShoot()));
    }

    /** Point turret at hub. */
    public static Command followHub(Turret turret, Swerve swerve, DoubleSupplier trimRight) {
        return turret.goToAngleFieldRelative(() -> {
            return AllianceFlipUtil.apply(FieldConstants.Hub.centerHub)
                .minus(swerve.state.getTurretCenterFieldFrame().getTranslation()).getAngle()
                .plus(Rotation2d.fromDegrees(trimRight.getAsDouble()));
        });
    }

    /** Reset the init */
    public static Command resetInit(Swerve swerve, Turret turret) {
        return Commands.runOnce(() -> {
            swerve.state.resetInit();
            turret.resetTurret();
        }).ignoringDisable(true);
    }
}
