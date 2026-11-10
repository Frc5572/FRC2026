package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
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
        assertEquals(FieldConstants.fieldWidth - pose1.getY(),
            AllianceFlipUtil.flipY(pose1).getY());
        assertEquals(pose1.getRotation().unaryMinus(), AllianceFlipUtil.flipY(pose1).getRotation());
        assertEquals(pose1.getX(), AllianceFlipUtil.flipY(pose1).getX());
        Pose2d pose2 = new Pose2d(1, 1, new Rotation2d(40));
        assertEquals(FieldConstants.fieldWidth - pose2.getY(),
            AllianceFlipUtil.flipY(pose2).getY());
        assertEquals(pose2.getRotation().unaryMinus(), AllianceFlipUtil.flipY(pose2).getRotation());
        assertEquals(pose2.getX(), AllianceFlipUtil.flipY(pose2).getX());
        Pose2d pose3 = new Pose2d(50, 50, new Rotation2d(0));
        assertEquals(FieldConstants.fieldWidth - pose3.getY(),
            AllianceFlipUtil.flipY(pose3).getY());
        assertEquals(pose3.getRotation().unaryMinus(), AllianceFlipUtil.flipY(pose3).getRotation());
        assertEquals(pose3.getX(), AllianceFlipUtil.flipY(pose3).getX());
        Pose2d pose4 = new Pose2d(50, 20, new Rotation2d(130));
        assertEquals(FieldConstants.fieldWidth - pose4.getY(),
            AllianceFlipUtil.flipY(pose4).getY());
        assertEquals(pose4.getRotation().unaryMinus(), AllianceFlipUtil.flipY(pose4).getRotation());
        assertEquals(pose4.getX(), AllianceFlipUtil.flipY(pose4).getX());

    }

    @Test
    public void testShouldFlip() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        assertTrue(AllianceFlipUtil.shouldFlip(),
            "shouldFlip should return true when alliance is red");

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        assertFalse(AllianceFlipUtil.shouldFlip(),
            "shouldFlip should return false when alliance is blue");
    }

    @Test
    public void testApplyX() {
        double x1 = 1;
        double x2 = 150;

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        assertEquals(FieldConstants.fieldLength - x1, AllianceFlipUtil.applyX(x1));
        assertEquals(FieldConstants.fieldLength - x2, AllianceFlipUtil.applyX(x2));

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        assertEquals(x1, AllianceFlipUtil.applyX(x1));
        assertEquals(x2, AllianceFlipUtil.applyX(x2));
    }

    @Test
    public void testApplyY() {
        double y1 = 1;
        double y2 = 150;

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        assertEquals(FieldConstants.fieldWidth - y1, AllianceFlipUtil.applyY(y1));
        assertEquals(FieldConstants.fieldWidth - y2, AllianceFlipUtil.applyY(y2));

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        assertEquals(y1, AllianceFlipUtil.applyY(y1));
        assertEquals(y2, AllianceFlipUtil.applyY(y2));
    }

    @Test
    public void testApplyRotation() {
        Rotation2d rotation1 = new Rotation2d(0);
        Rotation2d rotation2 = new Rotation2d(90);

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        assertEquals(rotation1.minus(Rotation2d.kPi), AllianceFlipUtil.apply(rotation1));
        assertEquals(rotation2.minus(Rotation2d.kPi), AllianceFlipUtil.apply(rotation2));

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        assertEquals(rotation1, AllianceFlipUtil.apply(rotation1));
        assertEquals(rotation2, AllianceFlipUtil.apply(rotation2));

    }
}
