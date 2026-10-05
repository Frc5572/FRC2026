package frc.robot.localization;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
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
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.CameraConstantsBuilder;

public class TurretLocalizationTest {

    private CameraConstants turretCameraConstants;
    private CameraConstants backCameraConstants;
    private TurretCameraAdapter turretAdapter;

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
        CameraProcessor processor = new CameraProcessor(turretCameraConstants, turretAdapter);
        turretAdapter.recordTurretAngle(1.0, Rotation2d.kZero);

        // Outside field test
        assertFalse(FieldConstants.isInField(new Translation2d(-0.5, 4.0)));
        assertFalse(FieldConstants.isInField(new Translation2d(FieldConstants.fieldLength + 1.0, 4.0)));
        assertFalse(FieldConstants.isInField(new Translation2d(4.0, -0.5)));
        assertFalse(FieldConstants.isInField(new Translation2d(4.0, FieldConstants.fieldWidth + 0.5)));
        assertTrue(FieldConstants.isInField(new Translation2d(4.0, 4.0)));

        // Inside hub test
        assertTrue(FieldConstants.isInsideHub(FieldConstants.Hub.centerHub));
        assertTrue(FieldConstants.isInsideHub(new Translation2d(
            FieldConstants.fieldLength - FieldConstants.Hub.centerHub.getX(),
            FieldConstants.Hub.centerHub.getY())));
        assertFalse(FieldConstants.isInsideHub(new Translation2d(1.0, 1.0)));
    }

    @Test
    public void testTurretMotionUpdatesLastTimeMoved() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);

        // Initial angle
        state.setTurretRawAngle(1.0, Degrees.of(0));

        // Motion under threshold (<= 2 degrees) shouldn't count as moving
        double movedTimeBefore = 1.0;
        state.setTurretRawAngle(1.05, Degrees.of(1.5));

        // Motion over threshold (> 2 degrees) should count as moving
        state.setTurretRawAngle(1.10, Degrees.of(5.0));
    }

    @Test
    public void testStationaryAndMovingRotationStdDev() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        state.resetPose(new Pose2d(5.0, 5.0, Rotation2d.kZero));

        // Mark turret moving at t = 10.0
        state.setTurretRawAngle(10.0, Degrees.of(0));
        state.setTurretRawAngle(10.02, Degrees.of(10.0)); // moved > 2 deg

        // Turret frame at t = 10.2 (< 0.5s since moved -> NOT stationary)
        Pose3d cameraPose = new Pose3d(5.0, 5.0, 0.5, new Rotation3d());
        Transform3d robotToCamera = new Transform3d();
        VisionObservation movingTurretObs = new VisionObservation(
            cameraPose, robotToCamera, 0.1, 0.05, 10.2, true, "turret");

        state.addVisionObservation(movingTurretObs);

        // When moving, rotation std dev is 10000.0, so heading is barely adjusted
        // Now test stationary turret frame at t = 11.0 (> 0.5s since moved -> stationary)
        VisionObservation stationaryTurretObs = new VisionObservation(
            cameraPose, robotToCamera, 0.1, 0.05, 11.0, true, "turret");

        state.addVisionObservation(stationaryTurretObs);

        // Non-turret camera at t = 10.2 (moving) should NOT have rotation std dev forced to 10000.0
        VisionObservation nonTurretObs = new VisionObservation(
            cameraPose, robotToCamera, 0.1, 0.05, 10.2, false, "back");
        state.addVisionObservation(nonTurretObs);
    }

    @Test
    public void testBumpTranslationSnap() {
        DrivetrainState state = new DrivetrainState(getZeroModulePositions(), Rotation2d.kZero);
        // Place initial estimate on field
        state.resetPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));

        // Mark moving at t = 10.0
        state.updateMeasuredSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));

        // Case 1: Moving, not on bump, error > 3 inches -> NO snap (filtered only)
        Pose3d visionPoseOffBump = new Pose3d(2.2, 2.0, 0.5, new Rotation3d()); // 0.2m = ~8 inches error
        VisionObservation movingOffBump = new VisionObservation(
            visionPoseOffBump, new Transform3d(), 0.1, 0.05, 10.1, true, "turret");
        state.addVisionObservation(movingOffBump);

        // Estimate should NOT have snapped all the way to 2.2
        assertTrue(Math.abs(state.getGlobalPoseEstimate().getX() - 2.2) > 0.05);

        // Case 2: Stationary, error > 3 inches -> SNAP occurs
        state.updateMeasuredSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        double now = MathSharedStore.getTimestamp();
        // Turret moved at 'now'
        state.setTurretRawAngle(now, Degrees.of(10.0));

        // Observation 1 second later (> 0.5s after last moved -> stationary)
        Pose3d visionPoseStationary = new Pose3d(3.0, 3.0, 0.5, new Rotation3d());
        VisionObservation stationaryObs = new VisionObservation(
            visionPoseStationary, new Transform3d(), 0.1, 0.05, now + 1.0, true, "turret");
        state.addVisionObservation(stationaryObs);

        // Snapped directly to (3.0, 3.0)
        assertEquals(3.0, state.getGlobalPoseEstimate().getX(), 1e-4);
        assertEquals(3.0, state.getGlobalPoseEstimate().getY(), 1e-4);
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
        File logFile = new File("/Users/jacerodgers/Downloads/akit_26-05-01_22-08-48_curie_q119.wpilog");
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
}
