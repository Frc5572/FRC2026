package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;

/**
 * Auto Command Factory
 */
public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    AdjustableHood adjustableHood;
    Climber climber;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    Turret turret;

    /**
     * Auto Command Factory
     */
    public AutoCommandFactory(AutoFactory autoFactory, Swerve swerve, AdjustableHood adjustableHood,
        Climber climber, Intake intake, Indexer indexer, Shooter shooter, Turret turret) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.adjustableHood = adjustableHood;
        this.climber = climber;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;
    }

    public AutoRoutine everything() {
        AutoRoutine routine = autoFactory.newRoutine("Everything");

        List<AutoAction> actions = new ArrayList<>();

        routine.active().onTrue(Commands.defer(() -> {
            Command command = Commands.runOnce(() -> {
            });
            boolean isFirst = true;

            for (var action : actions) {
                if (isFirst) {
                    command = action.command();
                    isFirst = false;
                } else {
                    command = command.andThen(action.command());
                }
            }

            return command;
        }, Set.of(swerve, adjustableHood, indexer, intake, shooter, turret)));

        return routine;
    }

    public static class AutoActionList implements Sendable, AutoCloseable {

        public ArrayList<AutoAction> actions = new ArrayList<>();

        @Override
        public void close() throws Exception {
            for (var item : actions) {
                item.close();
            }
        }

        @Override
        public void initSendable(SendableBuilder builder) {

        }

    }

    public static sealed interface AutoAction extends AutoCloseable {

        public Command command();

        public static final class Shoot implements AutoAction {

            @Override
            public Command command() {
                // TODO
                return Commands.runOnce(() -> {
                });
            }

            @Override
            public void close() throws Exception {
                // TODO
            }
        }

        public static final class IntakePath implements AutoAction {

            @Override
            public Command command() {
                // TODO
                return Commands.runOnce(() -> {
                });
            }

            @Override
            public void close() throws Exception {
                // TODO
            }
        }

        public static final class CrossTrench implements AutoAction {

            @Override
            public Command command() {
                // TODO
                return Commands.runOnce(() -> {
                });
            }

            @Override
            public void close() throws Exception {
                // TODO
            }
        }

        public static final class CrossBump implements AutoAction {

            @Override
            public Command command() {
                // TODO
                return Commands.runOnce(() -> {
                });
            }

            @Override
            public void close() throws Exception {
                // TODO
            }
        }

    }
}
