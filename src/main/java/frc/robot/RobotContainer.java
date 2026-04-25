package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.ToDoubleFunction;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.RobotRunType;
import frc.robot.commands.WaitSupplierCommand;
import frc.robot.sim.FuelSim;
import frc.robot.sim.SimulatedRobotState;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.adjustable_hood.AdjustableHoodIOEmpty;
import frc.robot.subsystems.adjustable_hood.AdjustableHoodReal;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOEmpty;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOEmpty;
import frc.robot.subsystems.indexer.IndexerReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOEmpty;
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOEmpty;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOEmpty;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.gyro.GyroIOEmpty;
import frc.robot.subsystems.swerve.gyro.GyroNavX2;
import frc.robot.subsystems.swerve.mod.SwerveModuleIOEmpty;
import frc.robot.subsystems.swerve.mod.SwerveModuleReal;
import frc.robot.subsystems.swerve.util.TeleopControls;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOEmpty;
import frc.robot.subsystems.turret.TurretReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOEmpty;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.tunable.ShotDataHelper;
import frc.robot.viz.RobotViz;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@NullMarked
public final class RobotContainer {
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);
    public final CommandXboxController tuner = new CommandXboxController(2);
    public final CommandXboxController pit = new CommandXboxController(3);

    /* Auto utilities */
    private final AutoChooser autoChooser = new AutoChooser();
    private final AutoCommandFactory autoCommandFactory;

    /* Subsystems */
    private final LEDs leds = new LEDs();
    private final Swerve swerve;
    private final Vision vision;
    private final AdjustableHood adjustableHood;
    private final Turret turret;
    private final Shooter shooter;
    private final Intake intake;
    private final Climber climber;
    private final Indexer indexer;
    private final RobotViz viz;
    private final SimulatedRobotState sim;
    private final Field2d field = new Field2d();
    // private final FieldObject2d autoJustShootLocation = field.getObject("Auto Just Shoot
    // Location");
    private final FieldObject2d autoStoppingPoint = field.getObject("Auto End Point");

    /**
     * Robot Container
     *
     * @param runtimeType Run type
     */
    public RobotContainer(RobotRunType runtimeType) {
        switch (runtimeType) {
            case kReal:
                sim = null;
                swerve = new Swerve(SwerveReal::new, GyroNavX2::new, SwerveModuleReal::new);
                vision = new Vision(swerve.state, new VisionReal());
                adjustableHood = new AdjustableHood(new AdjustableHoodReal());
                turret = new Turret(new TurretReal(), swerve.state);
                shooter = new Shooter(new ShooterReal(), swerve.state);
                intake = new Intake(new IntakeReal());
                climber = new Climber(new ClimberIOEmpty());
                indexer = new Indexer(new IndexerReal());

                break;
            case kSimulation:
                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                sim = new SimulatedRobotState(
                    new Pose2d(4.04, FieldConstants.fieldWidth - 0.7, Rotation2d.kCW_90deg));
                FuelSim.getInstance().registerRobot(Constants.Swerve.bumperFront.in(Meters) * 2,
                    Constants.Swerve.bumperRight.in(Meters), Units.inchesToMeters(5.0),
                    () -> sim.swerveDrive.mapleSim.getSimulatedDriveTrainPose(),
                    () -> sim.swerveDrive.mapleSim
                        .getDriveTrainSimulatedChassisSpeedsFieldRelative());
                FuelSim.getInstance().registerIntake(Constants.Swerve.bumperFront.in(Meters),
                    Constants.Swerve.bumperFront.in(Meters) + Units.inchesToMeters(7),
                    -Constants.Swerve.bumperRight.in(Meters),
                    Constants.Swerve.bumperRight.in(Meters), () -> sim.intake.isIntaking, () -> {
                        sim.indexer.addFuel();
                    });
                FuelSim.getInstance().start();
                swerve = new Swerve(sim.swerveDrive::simProvider, sim.swerveDrive::gyroProvider,
                    sim.swerveDrive::moduleProvider);
                vision = new Vision(swerve.state, sim.visionSim);
                adjustableHood = new AdjustableHood(sim.adjustableHood);
                turret = new Turret(sim.turret, swerve.state);
                shooter = new Shooter(sim.shooter, swerve.state);
                intake = new Intake(sim.intake);
                climber = new Climber(sim.climber);
                indexer = new Indexer(sim.indexer);

                SmartDashboard.putNumber("VisionFudge", 0.0);

                // FuelSim.getInstance().spawnStartingFuel();

                break;
            default:
                sim = null;
                swerve = new Swerve(SwerveIOEmpty::new, GyroIOEmpty::new, SwerveModuleIOEmpty::new);
                vision = new Vision(swerve.state, new VisionIOEmpty());
                adjustableHood = new AdjustableHood(new AdjustableHoodIOEmpty());
                turret = new Turret(new TurretIOEmpty(), swerve.state);
                shooter = new Shooter(new ShooterIOEmpty(), swerve.state);
                intake = new Intake(new IntakeIOEmpty());
                climber = new Climber(new ClimberSim());
                indexer = new Indexer(new IndexerIOEmpty());

                break;
        }
        // DASHBOARD STUFF
        SmartDashboard.putData(Constants.DashboardValues.autoChooser, autoChooser);
        SmartDashboard.putNumber(Constants.DashboardValues.shootX,
            Constants.DashboardValues.shootXDefault);
        SmartDashboard.putNumber(Constants.DashboardValues.shootY,
            Constants.DashboardValues.shootYDefault);
        SmartDashboard.putData(Constants.DashboardValues.field, field);
        SmartDashboard.putNumber(Constants.DashboardValues.feetPastCenter,
            Constants.DashboardValues.feetPastCenterDefault);
        SmartDashboard.putNumber(Constants.DashboardValues.x1, Constants.Auto.wilsonTestX);
        SmartDashboard.putNumber(Constants.DashboardValues.x2, Constants.Auto.wilsonTestX);
        SmartDashboard.putNumber(Constants.DashboardValues.delay,
            Constants.DashboardValues.delayDefault);
        SmartDashboard.putBoolean(Constants.DashboardValues.fullWidth, false);
        SmartDashboard.putBoolean(Constants.DashboardValues.shootFirst, false);
        SmartDashboard.putBoolean(Constants.DashboardValues.rampOrTrenchEnd, false);
        // END DASHBOARD STUFF

        viz = new RobotViz(sim, swerve, turret, adjustableHood, intake, climber, shooter);

        // AUTO STUFF
        autoCommandFactory = new AutoCommandFactory(swerve.autoFactory, swerve, adjustableHood,
            climber, intake, indexer, shooter, turret);
        // autoChooser.addRoutine("Gather then Shoot (Left)",
        // autoCommandFactory::gatherThenShootLeft);
        autoChooser.addRoutine(Constants.Auto.justShoot, autoCommandFactory::justShoot);
        autoChooser.addRoutine(Constants.Auto.wilsonTest, autoCommandFactory::wilsonTest);
        autoChooser.addRoutine("CMP Special", autoCommandFactory::cmpSpecial);
        // autoChooser.addRoutine(Constants.Auto.wilsonTestShort,
        // autoCommandFactory::wilsonTestShort);
        // autoChooser.addRoutine("wilsonTest2", autoCommandFactory::wilsonTest2);
        // Trigger isn't working for some reason during disabled mode, moved to disabled periodic
        // RobotModeTriggers.disabled().whileTrue(Commands.run(() -> {
        // double x = SmartDashboard.getNumber(Constants.DashboardValues.shootX, 0);
        // double y = SmartDashboard.getNumber(Constants.DashboardValues.shootY, 0);
        // autoJustShootLocation.setPose(x, y, new Rotation2d());
        // // System.out.println("asdfasdasdf");
        // // Logger.recordOutput("asdfadsf", autoJustShootLocation.getPose());
        // }));
        RobotModeTriggers.autonomous()
            .whileTrue(new WaitSupplierCommand(() -> SmartDashboard
                .getNumber(Constants.DashboardValues.delay, Constants.DashboardValues.delayDefault))
                    .andThen(autoChooser.selectedCommandScheduler())
                    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    .andThen(Commands.runOnce(() -> swerve.stop())));
        // END AUTO STUFF

        // DEFAULT COMMANDS
        adjustableHood.setDefaultCommand(adjustableHood.setGoal(Degrees.of(0)));
        turret.setDefaultCommand(turret
            .goToAngleFieldRelative(() -> swerve.state.getDesiredTurretHeadingFieldRelative()));
        leds.setDefaultCommand(leds.blinkLEDs(Color.kRed));
        swerve.setDefaultCommand(swerve.driveUserRelative(TeleopControls.teleopControls(
            () -> -combineControllers(CommandXboxController::getLeftY, driver, tuner),
            () -> -combineControllers(CommandXboxController::getLeftX, driver, tuner),
            () -> -combineControllers(CommandXboxController::getRightX, driver, tuner),
            Constants.DriverControls.driverTranslationalMaxSpeed,
            Constants.DriverControls.driverRotationalMaxSpeed)));
        shooter.setDefaultCommand(shooter.shoot(0.0));

        // TRIGGERS
        RobotModeTriggers.disabled().and(vision.seesTwoAprilTags.negate())
            .whileTrue(leds.setLEDsBreathe(Color.kBlue));
        RobotModeTriggers.teleop().onTrue(swerve.resetFieldRelativeOffsetBasedOnPose());
        // RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
        // swerve.state.setTrims(0.0, swerve.state.getTrimLeft());
        // }));
        vision.seesTwoAprilTags.whileTrue(leds.setRainbow());

        // BUTTON BINDINGS
        maybeController("Driver", driver, this::setupDriver);
        maybeController("Operator", operator, this::setupOperator);
        maybeController("Tuner", tuner, this::setupTuner);
        maybeController("Pit", pit, this::setupPit);
    }

    private double combineControllers(ToDoubleFunction<CommandXboxController> func,
        CommandXboxController... controllers) {
        double value = 0.0;
        for (var controller : controllers) {
            if (controller.isConnected()) {
                value += func.applyAsDouble(controller);
            }
        }
        return value;
    }

    private void setupDriver() {
        driver.y().onTrue(swerve.setFieldRelativeOffset());
        // driver.b().whileTrue(turret.goToAngleRobotRelative(() -> Rotation2d.kZero));
        driver.x().whileTrue(swerve.wheelsIn());

        driver.rightTrigger()
            .whileTrue(Commands.parallel(
                CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood),
                swerve.driveUserRelative(TeleopControls.teleopControls(
                    () -> -combineControllers(CommandXboxController::getLeftY, driver, tuner),
                    () -> -combineControllers(CommandXboxController::getLeftX, driver, tuner),
                    () -> -combineControllers(CommandXboxController::getRightX, driver, tuner),
                    Constants.DriverControls.driverTranslationalShootSpeed,
                    Constants.DriverControls.driverRotationalShootSpeed))));

        driver.povUp().onTrue(Commands.runOnce(() -> {
            swerve.state.incTrims(0.5, 0);
        }));
        driver.povDown().onTrue(Commands.runOnce(() -> {
            swerve.state.incTrims(-0.5, 0);
        }));
        driver.povLeft().onTrue(Commands.runOnce(() -> {
            // swerve.state.incTrims(0.0, 2.0);
        }));
        driver.povRight().onTrue(Commands.runOnce(() -> {
            // swerve.state.incTrims(0.0, -2.0);
        }));

        driver.leftTrigger().whileTrue(intake.extendHopper(1.0).andThen(intake.intakeBalls()))
            .onFalse(intake.retractHopper(0));

        driver.leftTrigger().and(driver.rightTrigger().negate())
            .whileTrue(indexer.spinWhileIntake());

        // driver.rightBumper()
        // .whileTrue(swerve.driveFacingSides(
        // () -> -combineControllers(CommandXboxController::getLeftY, driver, tuner),
        // () -> -combineControllers(CommandXboxController::getLeftX, driver, tuner),
        // Constants.DriverControls.driverTranslationalMaxSpeed,
        // Constants.DriverControls.driverRotationalShootSpeed));
        driver.rightBumper().whileTrue(swerve.toggleSideLock());
    }

    private void setupOperator() {
        operator.a().and(RobotModeTriggers.disabled())
            .onTrue(CommandFactory.resetInit(swerve, turret));
        operator.b().whileTrue(turret.setVoltage(() -> 0));
        operator.x().whileTrue(turret.setVoltage(() -> operator.getLeftY() * 3.0));
        operator.y().onTrue(Commands.runOnce(() -> swerve.state.setTrims(0.0, 0.0)));
        operator.povUp().onTrue(Commands.runOnce(() -> {
            swerve.state.incTrims(0.5, 0);
        }));
        operator.povDown().onTrue(Commands.runOnce(() -> {
            swerve.state.incTrims(-0.5, 0);
        }));
        operator.povLeft().onTrue(Commands.runOnce(() -> {
            swerve.state.incTrims(0.0, 2.0);
        }));
        operator.povRight().onTrue(Commands.runOnce(() -> {
            swerve.state.incTrims(0.0, -2.0);
        }));
    }

    private ShotDataHelper helper = new ShotDataHelper();

    private void setupTuner() {
        tuner.y().onTrue(swerve.setFieldRelativeOffset());

        tuner.rightTrigger()
            .whileTrue(shooter.shoot(() -> helper.flywheelSpeed)
                .alongWith(adjustableHood.setGoal(() -> Degrees.of(helper.hoodAngle))))
            .onFalse(shooter.shoot(0).alongWith(adjustableHood.setGoal(Degrees.of(0))));
        boolean[] firstShotFlag = {false};
        double[] firstShot = {0.0};
        tuner.leftTrigger().onTrue(Commands.runOnce(() -> {
            firstShotFlag[0] = true;
            Logger.recordOutput("ShotTiming/distance",
                Units.metersToFeet(AllianceFlipUtil.apply(swerve.state.getTurretCenterFieldFrame())
                    .getTranslation().getDistance(FieldConstants.Hub.centerHub)));
        })).whileTrue(indexer.setSpeedCommand(1.0, 1.0));
        new Trigger(() -> shooter.timeSinceLastShot() < 0.4).onTrue(Commands.runOnce(() -> {
            if (firstShotFlag[0]) {
                firstShot[0] = Timer.getFPGATimestamp() - shooter.timeSinceLastShot();
                Logger.recordOutput("ShotTiming/firstShot", firstShot[0]);
                firstShotFlag[0] = false;
            }
        }));

        tuner.a().onTrue(Commands.runOnce(() -> {
            double hitTarget = Timer.getFPGATimestamp();
            Logger.recordOutput("ShotTiming/hitTarget", hitTarget);
            Logger.recordOutput("ShotTiming/timeOfFlight", hitTarget - firstShot[0]);
        }));
    }

    private void setupPit() {
        pit.rightTrigger()
            .whileTrue(shooter.shoot(() -> helper.flywheelSpeed)
                .alongWith(adjustableHood.setGoal(() -> Degrees.of(helper.hoodAngle))))
            .onFalse(shooter.shoot(0).alongWith(adjustableHood.setGoal(Degrees.of(0))));
        pit.leftTrigger().whileTrue(indexer.setSpeedCommand(1.0, 1.0));
        pit.a().whileTrue(shooter.characterization()).onFalse(shooter.shoot(0));
        pit.b().whileTrue(intake.intakeBalls());
    }

    private List<Runnable> controllerSetups = new ArrayList<>();
    private final Set<String> seenController = new HashSet<>();

    private void maybeController(String name, CommandXboxController xboxController,
        Runnable setupFun) {
        Runnable runner = () -> {
            if (seenController.add(name)) {
                System.out.println("Setting up buttons for " + name);
                setupFun.run();
            }
        };
        if (xboxController.isConnected()) {
            runner.run();
        } else {
            new Trigger(xboxController::isConnected)
                .onTrue(Commands.runOnce(() -> controllerSetups.add(runner)).ignoringDisable(true));
        }
    }

    private void queryControllers() {
        for (var setup : controllerSetups) {
            setup.run();
        }
        controllerSetups.clear();
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {
        queryControllers();
        if (sim != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            FuelSim.getInstance().tick();
            sim.update();
        }
        viz.periodic();
        field.setRobotPose(swerve.state.getGlobalPoseEstimate());
    }


    /**
     * Runs during disabled
     */
    public void disabledPeriodic() {
        String selectedAuto =
            SmartDashboard.getString(Constants.DashboardValues.autoChooser + "/active", "");
        // System.out.println(selectedAuto);
        if (selectedAuto.equals(Constants.Auto.justShoot)) {
            double x = SmartDashboard.getNumber(Constants.DashboardValues.shootX,
                Constants.DashboardValues.shootXDefault);
            double y = SmartDashboard.getNumber(Constants.DashboardValues.shootY,
                Constants.DashboardValues.shootYDefault);
            autoStoppingPoint.setPose(AllianceFlipUtil.apply(new Pose2d(x, y, new Rotation2d())));

        } else if (selectedAuto.equals(Constants.Auto.wilsonTest)) {
            // System.out.println("asdf");
            Pose2d pose = AllianceFlipUtil.apply(new Pose2d(Constants.Auto.wilsonTestX,
                (FieldConstants.fieldWidth / 2.0) + Units
                    .feetToMeters(SmartDashboard.getNumber(Constants.DashboardValues.feetPastCenter,
                        Constants.DashboardValues.feetPastCenterDefault)),
                Rotation2d.kCCW_90deg));
            if (AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                .getY() > FieldConstants.fieldWidth / 2.0) {
                pose = AllianceFlipUtil.flipY(pose);
            }
            autoStoppingPoint.setPose(pose);
        } else if (selectedAuto.equals(Constants.Auto.wilsonTestShort)) {
            // System.out.println("asdf");
            Pose2d pose = AllianceFlipUtil.apply(new Pose2d(Constants.Auto.wilsonTestX2,
                (FieldConstants.fieldWidth / 2.0) + Units
                    .feetToMeters(SmartDashboard.getNumber(Constants.DashboardValues.feetPastCenter,
                        Constants.DashboardValues.feetPastCenterDefault)),
                Rotation2d.kCCW_90deg));
            if (AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                .getY() > FieldConstants.fieldWidth / 2.0) {
                pose = AllianceFlipUtil.flipY(pose);
            }
            autoStoppingPoint.setPose(pose);
        } else {
            autoStoppingPoint.setPoses(new ArrayList<Pose2d>());
        }
    }
}

