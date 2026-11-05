package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants;

public class AllianceFlipUtilTest {

    /**
     * Initializes the Hardware Abstraction Layer prior to running test suite.
     */
    @BeforeAll
    public static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testFlipY() {
        Pose2d pose1 = new Pose2d(1, 1, new Rotation2d(0));
        boolean pose1YFlipped =
            FieldConstants.fieldWidth - pose1.getY() == AllianceFlipUtil.flipY(pose1).getY();
        boolean pose1RotationFlipped = pose1.getRotation().minus(Rotation2d.kPi) == AllianceFlipUtil
            .flipY(pose1).getRotation();
        boolean pose1XUnchanged = pose1.getX() == AllianceFlipUtil.flipY(pose1).getX();
        Pose2d pose2 = new Pose2d(1, 1, new Rotation2d(40));
        boolean pose2YFlipped =
            FieldConstants.fieldWidth - pose2.getY() == AllianceFlipUtil.flipY(pose2).getY();
        boolean pose2RotationFlipped = pose2.getRotation().minus(Rotation2d.kPi) == AllianceFlipUtil
            .flipY(pose2).getRotation();
        boolean pose2XUnchanged = pose2.getX() == AllianceFlipUtil.flipY(pose2).getX();
        Pose2d pose3 = new Pose2d(50, 50, new Rotation2d(0));
        boolean pose3YFlipped =
            FieldConstants.fieldWidth - pose3.getY() == AllianceFlipUtil.flipY(pose3).getY();
        boolean pose3RotationFlipped = pose3.getRotation().minus(Rotation2d.kPi) == AllianceFlipUtil
            .flipY(pose3).getRotation();
        boolean pose3XUnchanged = pose3.getX() == AllianceFlipUtil.flipY(pose3).getX();
        Pose2d pose4 = new Pose2d(50, 20, new Rotation2d(130));
        boolean pose4YFlipped =
            FieldConstants.fieldWidth - pose4.getY() == AllianceFlipUtil.flipY(pose4).getY();
        boolean pose4RotationFlipped = pose4.getRotation().minus(Rotation2d.kPi) == AllianceFlipUtil
            .flipY(pose4).getRotation();
        boolean pose4XUnchanged = pose4.getX() == AllianceFlipUtil.flipY(pose4).getX();

        assertTrue(pose1YFlipped && pose2YFlipped && pose3YFlipped && pose4YFlipped,
            "Y values should be flipped");
        assertTrue(
            pose1RotationFlipped && pose2RotationFlipped && pose3RotationFlipped
                && pose4RotationFlipped,
            "Rotation values should be flipped unless they are 0 degrees");
        assertTrue(pose1XUnchanged && pose2XUnchanged && pose3XUnchanged && pose4XUnchanged,
            "X values should remain unchanged");
    }

    @Test
    public void testShouldFlip() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationDataJNI.notifyNewData();
        assertTrue(AllianceFlipUtil.shouldFlip(),
            "shouldFlip should return true when alliance is red");

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationDataJNI.notifyNewData();
        assertFalse(AllianceFlipUtil.shouldFlip(),
            "shouldFlip should return false when alliance is blue");
    }

    @Test
    public void testApplyX() {
        double x1 = 1;
        double x2 = 150;

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);

        boolean x1Flipped = FieldConstants.fieldLength - x1 == AllianceFlipUtil.applyX(x1);
        boolean x2Flipped = FieldConstants.fieldLength - x2 == AllianceFlipUtil.applyX(x2);

        assertTrue(x1Flipped && x2Flipped, "X values should be flipped when alliance is red");

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        boolean x1Unchanged = x1 == AllianceFlipUtil.applyX(x1);
        boolean x2Unchanged = x2 == AllianceFlipUtil.applyX(x2);

        assertTrue(x1Unchanged && x2Unchanged,
            "X values should remain unchanged when alliance is blue");
    }

    @Test
    public void testApplyY() {
        double y1 = 1;
        double y2 = 150;

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);

        boolean y1Flipped = FieldConstants.fieldWidth - y1 == AllianceFlipUtil.applyY(y1);
        boolean y2Flipped = FieldConstants.fieldWidth - y2 == AllianceFlipUtil.applyY(y2);

        assertTrue(y1Flipped && y2Flipped, "Y values should be flipped when alliance is red");

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        boolean y1Unchanged = y1 == AllianceFlipUtil.applyY(y1);
        boolean y2Unchanged = y2 == AllianceFlipUtil.applyY(y2);

        assertTrue(y1Unchanged && y2Unchanged,
            "Y values should remain unchanged when alliance is blue");
    }

    @Test
    public void testApplyRotation() {
        Rotation2d rotation1 = new Rotation2d(0);
        Rotation2d rotation2 = new Rotation2d(90);

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);

        boolean rotation1Flipped =
            rotation1.minus(Rotation2d.kPi) == AllianceFlipUtil.apply(rotation1);
        boolean rotation2Flipped =
            rotation2.minus(Rotation2d.kPi) == AllianceFlipUtil.apply(rotation2);

        assertTrue(rotation1Flipped && rotation2Flipped,
            "Rotation values should be flipped when alliance is red");

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        boolean rotation1Unchanged = rotation1 == AllianceFlipUtil.apply(rotation1);
        boolean rotation2Unchanged = rotation2 == AllianceFlipUtil.apply(rotation2);

        assertTrue(rotation1Unchanged && rotation2Unchanged,
            "Rotation values should remain unchanged when alliance is blue");
    }
}
