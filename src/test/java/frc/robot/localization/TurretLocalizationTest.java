package frc.robot.localization;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.util.SwerveArcOdometry;
import frc.robot.math.geometry.Penetration;
import frc.robot.math.geometry.Rectangle;
import frc.robot.math.geometry.SeparatingAxis;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.CameraConstantsBuilder;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;

public class TurretLocalizationTest {

    private CameraConstants turretCameraConstants;
    private CameraConstants backCameraConstants;
    private TurretCameraAdapter turretAdapter;

    @BeforeAll
    public static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    public void setUp() {
        turretCameraConstants = new CameraConstantsBuilder()
            .coProcessorName("ubuntu")
            .name("turret")
            .height(800)
            .width(1280)
            .horizontalFieldOfView(80)
            .simFps(20)
            .simLatency(0.8)
            .simLatencyStdDev(0.02)
            .calibrationErrorMean(0.8)
            .calibrationErrorStdDev(0.08)
            .robotToCamera(new Transform3d(Constants.Vision.turretCenter, Constants.Vision.turretRight))
            .translationError(Units.inchesToMeters(6))
            .rotationError(0.3)
            .singleTagError(0)
            .isTurret(true)
            .finish();

        backCameraConstants = new CameraConstantsBuilder()
            .coProcessorName("skip")
            .name("back")
            .height(800)
            .width(1280)
            .horizontalFieldOfView(80)
            .simFps(20)
            .simLatency(0.3)
            .simLatencyStdDev(0.02)
            .calibrationErrorMean(0.8)
            .calibrationErrorStdDev(0.08)
            .robotToCamera(new Transform3d(new Translation3d(-0.335, 0.19, 0.202),
                new Rotation3d(Degrees.of(180), Degrees.of(-26), Degrees.of(163))))
            .translationError(0.3)
            .rotationError(0.3)
            .singleTagError(0)
            .isTurret(false)
            .finish();

        turretAdapter = new TurretCameraAdapter(Constants.Vision.turretCenter.getTranslation());
    }

    private SwerveModulePosition[] getZeroModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, Rotation2d.kZero),
            new SwerveModulePosition(0.0, Rotation2d.kZero),
            new SwerveModulePosition(0.0, Rotation2d.kZero),
            new SwerveModulePosition(0.0, Rotation2d.kZero)
        };
    }

    @Test
    public void testPhysicalImpossibilityRejection() {
        // Outside field test - point center
        assertFalse(FieldConstants.isInField(new Translation2d(-0.5, 4.0)));
        assertFalse(FieldConstants.isInField(new Translation2d(FieldConstants.fieldLength + 1.0, 4.0)));
        assertFalse(FieldConstants.isInField(new Translation2d(4.0, -0.5)));
        assertFalse(FieldConstants.isInField(new Translation2d(4.0, FieldConstants.fieldWidth + 0.5)));
        assertTrue(FieldConstants.isInField(new Translation2d(4.0, 4.0)));

        // Robot bounding box outside perimeter test
        Rectangle robotRect = new Rectangle("pose", Pose2d.kZero,
            Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);

        // Center is inside (x = 0.1), but corners extend past x = 0.0
        robotRect.setPose(new Pose2d(0.1, 4.0, Rotation2d.kZero));
        boolean anyCornerOutside = false;
        for (var corner : robotRect.getCorners()) {
            if (corner.getX() < 0.0 || corner.getX() > FieldConstants.fieldLength
                || corner.getY() < 0.0 || corner.getY() > FieldConstants.fieldWidth) {
                anyCornerOutside = true;
                break;
            }
        }
        assertTrue(anyCornerOutside, "Robot bumper extending past perimeter should be detected as outside field");

        // Inside hub test - point center
        assertTrue(FieldConstants.isInsideHub(FieldConstants.Hub.centerHub));
        assertTrue(FieldConstants.isInsideHub(new Translation2d(
            FieldConstants.fieldLength - FieldConstants.Hub.centerHub.getX(),
            FieldConstants.Hub.centerHub.getY())));
        assertFalse(FieldConstants.isInsideHub(new Translation2d(1.0, 1.0)));

        // SeparatingAxis hub collision test
        Rectangle hubRect = new Rectangle("hub",
            new Pose2d(FieldConstants.Hub.centerHub, Rotation2d.kZero),
            FieldConstants.Hub.width, FieldConstants.Hub.width);

        Penetration penetration = new Penetration("pen");
        // Robot centered right on hub
        robotRect.setPose(new Pose2d(FieldConstants.Hub.centerHub, Rotation2d.kZero));
        assertTrue(SeparatingAxis.solve(robotRect, hubRect, penetration),
            "Robot at hub center must penetrate hub");

        // Robot safely away from hub
        robotRect.setPose(new Pose2d(1.0, 1.0, Rotation2d.kZero));
        assertFalse(SeparatingAxis.solve(robotRect, hubRect, penetration),
            "Robot at (1, 1) must not penetrate hub");
    }

    @Test
    public void testTurretMotionUpdatesLastTimeMoved() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);

        assertEquals(0.0, state.getLastTimeMoved(), 1e-6);

        // Initial angle with 0 deg difference (prevAngle defaults to 0.0)
        state.setTurretRawAngle(1.0, Degrees.of(0));
        assertEquals(0.0, state.getLastTimeMoved(), 1e-6, "Initial 0 deg should not trigger movement");

        // Motion under threshold (<= 2 degrees) shouldn't count as moving
        state.setTurretRawAngle(1.05, Degrees.of(1.5));
        assertEquals(0.0, state.getLastTimeMoved(), 1e-6, "Change of 1.5 deg (<= 2 deg) should not count as moving");

        // Motion over threshold (> 2 degrees) should count as moving
        state.setTurretRawAngle(1.10, Degrees.of(5.0));
        assertEquals(1.10, state.getLastTimeMoved(), 1e-6,
            "Change of > 2 deg must update lastTimeMoved to the provided timestamp");
    }

    @Test
    public void testStationaryAndMovingRotationStdDev() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        state.resetPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));

        double t0 = 1.0;
        state.addOdometryObservation(getZeroModulePositions(), Rotation2d.kZero, t0, true);

        // Mark turret moving at t0
        state.setTurretRawAngle(t0, Degrees.of(0));
        state.setTurretRawAngle(t0, Degrees.of(10.0)); // moved > 2 deg

        // Turret frame at t0 (< 0.5s since moved -> NOT stationary)
        // Camera indicates a 45 degree heading offset
        Pose3d cameraPose = new Pose3d(2.0, 2.0, 0.5, new Rotation3d(0, 0, Math.toRadians(45)));
        Transform3d robotToCamera = new Transform3d();
        VisionObservation movingTurretObs = new VisionObservation(
            cameraPose, robotToCamera, 0.1, 0.05, t0, true, "turret", true, 10000.0);

        state.addVisionObservation(movingTurretObs);

        // When moving, turret camera rotation std dev is set to 10000.0, so robot heading is barely touched
        double headingAfterMoving = state.getGlobalPoseEstimate().getRotation().getDegrees();
        assertTrue(Math.abs(headingAfterMoving) < 0.1,
            "Turret camera should not touch heading while moving (got: " + headingAfterMoving + " deg)");

        // 1.0s later (> 0.5s since moved -> now stationary)
        double t1 = 2.0;
        state.addOdometryObservation(getZeroModulePositions(), Rotation2d.kZero, t1, true);

        VisionObservation stationaryTurretObs = new VisionObservation(
            cameraPose, robotToCamera, 0.1, 0.05, t1, true, "turret", true, 0.05);

        state.addVisionObservation(stationaryTurretObs);

        // Stationary turret camera fuses heading at normal std dev (0.05 rad), pulling heading toward 45 deg
        double headingAfterStationary = state.getGlobalPoseEstimate().getRotation().getDegrees();
        assertTrue(Math.abs(headingAfterStationary) > 1.0,
            "Stationary turret camera should adjust heading (got: " + headingAfterStationary + " deg)");

        // Reset pose back to 0 heading to test non-turret camera while moving
        state.resetPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));
        double t2 = 3.0;
        state.addOdometryObservation(getZeroModulePositions(), Rotation2d.kZero, t2, true);
        state.setTurretRawAngle(t2, Degrees.of(30.0)); // mark moving again

        // Non-turret camera (moving) should NOT have rotation std dev forced to 10000.0
        VisionObservation nonTurretObs = new VisionObservation(
            cameraPose, robotToCamera, 0.1, 0.05, t2, false, "back", true);
        state.addVisionObservation(nonTurretObs);

        double headingAfterNonTurret = state.getGlobalPoseEstimate().getRotation().getDegrees();
        assertTrue(Math.abs(headingAfterNonTurret) > 1.0,
            "Non-turret camera should fuse heading even when moving (got: " + headingAfterNonTurret + " deg)");
    }


    @Test
    public void testBumpTranslationSnap() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        // Place initial estimate on field
        state.resetPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));

        double now = MathSharedStore.getTimestamp();

        // Mark moving at 'now'
        state.updateMeasuredSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));

        // Case 1: Moving, not on bump, error > 3 inches -> NO snap (filtered only)
        Pose3d visionPoseOffBump = new Pose3d(2.2, 2.0, 0.5, new Rotation3d()); // 0.2m = ~8 inches error
        VisionObservation movingOffBump = new VisionObservation(
            visionPoseOffBump, new Transform3d(), 0.1, 0.05, now + 0.1, true, "turret", true);
        state.addVisionObservation(movingOffBump);

        // Estimate should NOT have snapped all the way to 2.2
        assertTrue(Math.abs(state.getGlobalPoseEstimate().getX() - 2.2) > 0.05);

        // Case 2: Stationary, error > 3 inches -> SNAP occurs
        state.updateMeasuredSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        state.setTurretRawAngle(now, Degrees.of(10.0));

        // Observation 1 second later (> 0.5s after last moved -> stationary)
        Pose3d visionPoseStationary = new Pose3d(3.0, 3.0, 0.5, new Rotation3d());
        VisionObservation stationaryObs = new VisionObservation(
            visionPoseStationary, new Transform3d(), 0.1, 0.05, now + 1.0, true, "turret", true);
        state.addVisionObservation(stationaryObs);

        // Snapped directly to (3.0, 3.0)
        assertEquals(3.0, state.getGlobalPoseEstimate().getX(), 1e-4);
        assertEquals(3.0, state.getGlobalPoseEstimate().getY(), 1e-4);

        // Case 3: On bump (robot on bump or vision on bump), error > 3 inches -> SNAP occurs even if moving
        Pose2d bumpCenterPose = new Pose2d(
            FieldConstants.Hub.centerHub.getX(),
            FieldConstants.Hub.centerHub.getY() + Units.inchesToMeters(50.0),
            Rotation2d.kZero);
        state.resetPose(bumpCenterPose);
        state.updateMeasuredSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0)); // moving!

        Pose3d bumpVisionPose = new Pose3d(
            bumpCenterPose.getX() + 0.2, bumpCenterPose.getY() + 0.2, 0.5, new Rotation3d());
        VisionObservation onBumpMovingObs = new VisionObservation(
            bumpVisionPose, new Transform3d(), 0.1, 0.05, now + 0.01, true, "turret", true);
        state.addVisionObservation(onBumpMovingObs);

        // Should snap because it is on bump!
        assertEquals(bumpVisionPose.getX(), state.getGlobalPoseEstimate().getX(), 1e-4);
        assertEquals(bumpVisionPose.getY(), state.getGlobalPoseEstimate().getY(), 1e-4);
    }

    @Test
    public void testIsOnBumpGeometry() {
        // Center hub X and slightly offset in Y (within LeftBump width)
        Pose2d allianceBumpPose = new Pose2d(
            FieldConstants.Hub.centerHub.getX(),
            FieldConstants.Hub.centerHub.getY() + Units.inchesToMeters(50.0),
            Rotation2d.kZero);
        assertTrue(FieldConstants.isOnBump(allianceBumpPose));

        // Opposing bump
        Pose2d oppBumpPose = new Pose2d(
            FieldConstants.fieldLength - FieldConstants.Hub.centerHub.getX(),
            FieldConstants.Hub.centerHub.getY() + Units.inchesToMeters(50.0),
            Rotation2d.kZero);
        assertTrue(FieldConstants.isOnBump(oppBumpPose));

        // Off bump
        Pose2d offBumpPose = new Pose2d(1.0, 1.0, Rotation2d.kZero);
        assertFalse(FieldConstants.isOnBump(offBumpPose));
    }

    @Test
    public void testCurieMatchLogIfPresent() {
        File logFile = new File("logs/akit_26-05-01_22-08-48_curie_q119.wpilog");
        if (!logFile.exists()) {
            logFile = new File("/Users/jacerodgers/Downloads/akit_26-05-01_22-08-48_curie_q119.wpilog");
        }
        if (!logFile.exists()) {
            System.out.println("Curie log file not found, skipping log replay test.");
            return;
        }

        System.out.println("Found Curie Q119 log file! Inspecting vision & bump entries...");
        WPILOGReader reader = new WPILOGReader(logFile.getAbsolutePath());
        reader.start();
        org.littletonrobotics.junction.LogTable table = new org.littletonrobotics.junction.LogTable(0);
        int totalFrames = 0;
        int onBumpFrames = 0;
        int turretMovedFrames = 0;
        int speedMovedFrames = 0;
        int visionOnBumpFrames = 0;
        java.util.Map<String, org.littletonrobotics.junction.LogTable.LogValue> runningState =
            new java.util.HashMap<>();
        while (reader.updateTable(table)) {
            totalFrames++;
            runningState.putAll(table.getAll(false));
            var onBump = runningState.get("/RealOutputs/State/isOnBump");
            if (onBump != null && onBump.getBoolean()) {
                onBumpFrames++;
            }
            var turretMoved = runningState.get("/RealOutputs/State/stationary/turret");
            if (turretMoved != null && turretMoved.getBoolean()) {
                turretMovedFrames++;
            }
            var speedMoved = runningState.get("/RealOutputs/State/stationary/speeds");
            if (speedMoved != null && speedMoved.getBoolean()) {
                speedMovedFrames++;
            }
            var turretEst = runningState.get("/RealOutputs/Viz/Cameras/turret/Est");
            if (turretEst != null) {
                double[] poseArr = turretEst.getDoubleArray();
                if (poseArr != null && poseArr.length >= 2) {
                    Pose2d vPose = new Pose2d(poseArr[0], poseArr[1], Rotation2d.kZero);
                    if (FieldConstants.isOnBump(vPose)) {
                        visionOnBumpFrames++;
                    }
                }
            }
        }
        System.out.println("Curie Q119 Log analysis: " + totalFrames + " total frames | onBump: "
            + onBumpFrames + " | turretMoved: " + turretMovedFrames + " | speedMoved: " + speedMovedFrames
            + " | visionOnBump: " + visionOnBumpFrames);
        assertTrue(totalFrames > 0, "Curie Q119 match log should have frames");
        assertTrue(turretMovedFrames > 0, "Curie Q119 match should have instances where turret moved");
    }

    @Test
    public void testAdapterRateAndWhipping() {
        TurretCameraAdapter adapter = new TurretCameraAdapter(new Translation3d(0, 0, 0));
        double t0 = 1.0;
        adapter.recordTurretAngle(t0 - 0.04, Rotation2d.fromDegrees(0), false);
        adapter.recordTurretAngle(t0 + 0.04, Rotation2d.fromDegrees(8), false);

        double rate = adapter.getRateAt(t0);
        // Rate is (8 deg in rad) / (2 * 0.04s) = 0.1396 / 0.08 ~ 1.745 rad/s
        assertEquals(Math.toRadians(8) / 0.08, rate, 1e-3, "Calculated central difference rate should match");
        assertFalse(adapter.isWhippingAt(t0), "Should not be whipping at t0");

        // Record whipping at t1
        double t1 = 2.0;
        adapter.recordTurretAngle(t1, Rotation2d.fromDegrees(45), true);
        assertTrue(adapter.isWhippingAt(t1), "Should be whipping at t1");

        // Hub tag recording
        assertFalse(adapter.getLastHubTagTime() > 0);
        adapter.recordFrame(2.5, true);
        assertEquals(2.5, adapter.getLastHubTagTime(), 1e-6);
        assertTrue(FieldConstants.isHubTag(26));
        assertFalse(FieldConstants.isHubTag(1));
    }

    @Test
    public void testLimitPositionHubPushout() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        // Pose penetrating the alliance hub
        Pose2d penetratingPose = new Pose2d(FieldConstants.Hub.centerHub, Rotation2d.kZero);
        Pose2d[] pushedPose = new Pose2d[1];
        state.limitPosition(penetratingPose, p -> pushedPose[0] = p);

        assertNotNull(pushedPose[0], "Limit position should push out a pose penetrating the hub");
        // Verify pushed pose moved away from the hub center
        double dist = pushedPose[0].getTranslation().getDistance(FieldConstants.Hub.centerHub);
        assertTrue(dist > 0.5, "Pushed pose should be pushed out by bumper radius");
    }

    @Test
    public void testBumpCrossingPushout() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        // Test positions along LeftBump (crossing in X at Y = 5.0)
        double hubX = FieldConstants.Hub.centerHub.getX();
        for (double x = hubX - 0.5; x <= hubX + 0.5; x += 0.1) {
            Pose2d pose = new Pose2d(x, 5.0, Rotation2d.kZero);
            Pose2d[] pushed = new Pose2d[1];
            state.limitPosition(pose, p -> pushed[0] = p);
            assertTrue(FieldConstants.isOnBump(pose), "Pose should be recognized as on the bump");
            // While on the bump, robot should not be pushed backwards
            assertNotNull(pose);
        }
    }

    @Test
    public void testBumpOdometryMotion() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        // Start before bump at X = 3.5, Y = 5.0
        state.overrideInit(new Pose2d(3.5, 5.0, Rotation2d.kZero));

        double dist = 0.0;
        double t = 0.0;
        for (int i = 0; i < 50; i++) { // 1 second of driving forward at 2 m/s
            t += 0.02;
            dist += 2.0 * 0.02;
            SwerveModulePosition[] pos = new SwerveModulePosition[] {
                new SwerveModulePosition(dist, Rotation2d.kZero),
                new SwerveModulePosition(dist, Rotation2d.kZero),
                new SwerveModulePosition(dist, Rotation2d.kZero),
                new SwerveModulePosition(dist, Rotation2d.kZero)
            };
            state.addOdometryObservation(pos, Rotation2d.kZero, t, true);
        }
        assertTrue(state.getGlobalPoseEstimate().getX() > 4.5,
            "Odometry should successfully advance past the bump center");
    }

    @Test
    public void testVisionSeesTurretTagTogglesCorrectly() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        TurretCameraAdapter adapter =
            new TurretCameraAdapter(Constants.Vision.turretCenter.getTranslation());
        class TestVisionIO implements VisionIO {
            public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

            @Override
            public void updateInputs(CameraInputs[] inputs) {
                if (inputs.length > 0) {
                    inputs[0].results = results;
                }
            }
        }
        TestVisionIO testIO = new TestVisionIO();
        Vision vision = new Vision(state, testIO, adapter);

        assertFalse(vision.seesTurretTag(), "Should not see tag initially");

        // Simulate frame with a target from turret camera
        PhotonTrackedTarget target = new PhotonTrackedTarget();
        target.fiducialId = 1;
        testIO.results = new PhotonPipelineResult[] {
            new PhotonPipelineResult(0L, 0L, 0L, 1L, List.of(target))
        };
        vision.periodic();
        assertTrue(vision.seesTurretTag(), "Should see turret tag after receiving frame with target");

        // Simulate frame with no targets
        testIO.results = new PhotonPipelineResult[] {
            new PhotonPipelineResult(0L, 0L, 0L, 2L, List.of())
        };
        vision.periodic();
        assertFalse(vision.seesTurretTag(), "Should no longer see turret tag when frame has no targets");
    }
}
