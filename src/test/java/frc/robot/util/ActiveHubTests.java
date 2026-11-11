package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class ActiveHubTests {

    /**
     * Initializes the Hardware Abstraction Layer prior to running test suite.
     */
    @BeforeAll
    public static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @Test
    public void testTimeLeftInCurrentPhase() {
        DriverStationSim.setMatchTime(140);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 10);

        DriverStationSim.setMatchTime(135);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 5);

        DriverStationSim.setMatchTime(130.0000000001);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 1E-10, 1E-3);

        DriverStationSim.setMatchTime(130);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 25);

        DriverStationSim.setMatchTime(115);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 10);

        DriverStationSim.setMatchTime(105.0000000001);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 1E-10, 1E-3);

        DriverStationSim.setMatchTime(105);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 25);

        DriverStationSim.setMatchTime(90);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 10);

        DriverStationSim.setMatchTime(80.0000000001);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 1E-10, 1E-3);

        DriverStationSim.setMatchTime(80);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 25);

        DriverStationSim.setMatchTime(65);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 10);

        DriverStationSim.setMatchTime(55.0000000001);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 1E-10, 1E-3);

        DriverStationSim.setMatchTime(55);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 25);

        DriverStationSim.setMatchTime(40);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 10);

        DriverStationSim.setMatchTime(30.0000000001);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 1E-10, 1E-3);

        DriverStationSim.setMatchTime(30);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 30);

        DriverStationSim.setMatchTime(10);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 10);

        DriverStationSim.setMatchTime(0.0000000001);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 1E-10, 1E-3);

        DriverStationSim.setMatchTime(0);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.timeLeftInCurrentPhase(), 0);
    }

    @Test
    public void testCurrentHubIsActive() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.setGameSpecificMessage("R");
        DriverStationSim.notifyNewData();
        testCurrentHubIsActiveHelper(true, true);

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.setGameSpecificMessage("B");
        DriverStationSim.notifyNewData();
        testCurrentHubIsActiveHelper(true, false);

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setGameSpecificMessage("B");
        DriverStationSim.notifyNewData();
        testCurrentHubIsActiveHelper(false, false);

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setGameSpecificMessage("R");
        DriverStationSim.notifyNewData();
        testCurrentHubIsActiveHelper(false, true);
    }

    private void testCurrentHubIsActiveHelper(boolean isRed, boolean redFirst) {
        DriverStationSim
            .setAllianceStationId(isRed ? AllianceStationID.Red1 : AllianceStationID.Blue1);
        DriverStationSim.setGameSpecificMessage(redFirst ? "R" : "B");
        DriverStationSim.notifyNewData();

        // Transition Phase (Both Hubs Should Be Active)
        testCurrentHubIsActiveTimeHelper(140, true, "Both Hubs Should be Active");

        testCurrentHubIsActiveTimeHelper(135, true, "Both Hubs Should be Active");

        testCurrentHubIsActiveTimeHelper(130.0000000001, true, "Both Hubs Should be Active");

        // First Phase (First Hub Active)
        testCurrentHubIsActiveTimeHelper(130, isRed == redFirst || !isRed == !redFirst,
            "First Hub Should be Active: %s (First Phase)"
                .formatted(isRed == redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(115, isRed == redFirst || !isRed == !redFirst,
            "First Hub Should be Active: %s (First Phase)"
                .formatted(isRed == redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(105.0000000001, isRed == redFirst || !isRed == !redFirst,
            "First Hub Should be Active: %s (First Phase)"
                .formatted(isRed == redFirst ? "Red" : "Blue"));

        // Second Phase (Second Hub Active)
        testCurrentHubIsActiveTimeHelper(105, isRed == !redFirst || !isRed == redFirst,
            "Second Hub Should be Active: %s (Second Phase)"
                .formatted(isRed == !redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(90, isRed == !redFirst || !isRed == redFirst,
            "Second Hub Should be Active: %s (Second Phase)"
                .formatted(isRed == !redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(80.0000000001, isRed == !redFirst || !isRed == redFirst,
            "Second Hub Should be Active: %s (Second Phase)"
                .formatted(isRed == !redFirst ? "Red" : "Blue"));

        // Third Phase (First Hub Active)
        testCurrentHubIsActiveTimeHelper(80, isRed == redFirst || !isRed == !redFirst,
            "First Hub Should be Active: %s (Third Phase)"
                .formatted(isRed == redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(65, isRed == redFirst || !isRed == !redFirst,
            "First Hub Should be Active: %s (Third Phase)"
                .formatted(isRed == redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(55.0000000001, isRed == redFirst || !isRed == !redFirst,
            "First Hub Should be Active: %s (Third Phase)"
                .formatted(isRed == redFirst ? "Red" : "Blue"));

        // Fourth Phase (Second Hub Active)
        testCurrentHubIsActiveTimeHelper(55, isRed == !redFirst || !isRed == redFirst,
            "Second Hub Should be Active: %s (Fourth Phase)"
                .formatted(isRed == !redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(40, isRed == !redFirst || !isRed == redFirst,
            "Second Hub Should be Active: %s (Fourth Phase)"
                .formatted(isRed == !redFirst ? "Red" : "Blue"));

        testCurrentHubIsActiveTimeHelper(30.0000000001, isRed == !redFirst || !isRed == redFirst,
            "Second Hub Should be Active: %s (Fourth Phase)"
                .formatted(isRed == !redFirst ? "Red" : "Blue"));

        // End Game (Both Hubs Should Be Active)
        testCurrentHubIsActiveTimeHelper(30, true, "Both Hubs Should be Active (End Game)");

        testCurrentHubIsActiveTimeHelper(10, true, "Both Hubs Should be Active (End Game)");

        testCurrentHubIsActiveTimeHelper(1E-10, true, "Both Hubs Should be Active (End Game)");

        testCurrentHubIsActiveTimeHelper(0, true, "Both Hubs Should be Active (End Game)");
    }

    private void testCurrentHubIsActiveTimeHelper(double time, boolean expected, String message) {
        DriverStationSim.setMatchTime(time);
        DriverStationSim.notifyNewData();
        assertEquals(ActiveHub.currentHubIsActive(), expected, message);
    }
}
