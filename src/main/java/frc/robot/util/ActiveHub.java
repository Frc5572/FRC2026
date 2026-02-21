package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for tracking which hub is active */
public class ActiveHub {

    public static Trigger activeHub = new Trigger(() -> checkHub(activeHubAfterAuto()));

    public ActiveHub() {}

    private static HubState activeHubAfterAuto() {
        var gamedata = DriverStation.getGameSpecificMessage();
        if (gamedata.length() > 0) {
            switch (gamedata.charAt(0)) {
                case 'B':
                    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
                        return HubState.FIRST;
                    } else {
                        return HubState.SECOND;
                    }
                case 'R':
                    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
                        return HubState.FIRST;
                    } else {
                        return HubState.SECOND;
                    }
                default:
                    return HubState.FIRST;
            }
        } else {
            return HubState.NODATA;
        }
    }

    private static boolean checkHub(HubState state) {
        var time = DriverStation.getMatchTime();
        if (state == HubState.FIRST) {
            if (time <= 30.0) {
                return true;
            }
            // 0:55 remaining
            if (time <= 55.0) {
                return false;
            }
            // 1:20 remaining
            if (time <= 80.0) {
                return true;
            }
            // 1:45 remaining
            if (time <= 105.0) {
                return false;
            }
            // 2:10 remaining
            if (time <= 130.0) {
                return true;
            }
            // 2:20 remaining
            if (time <= 140) {
                return true;
            }
            return false;
        } else if (state == HubState.SECOND) {
            if (time <= 30.0) {
                return true;
            }
            // 0:55 remaining
            if (time <= 55.0) {
                return true;
            }
            // 1:20 remaining
            if (time <= 80.0) {
                return false;
            }
            // 1:45 remaining
            if (time <= 105.0) {
                return true;
            }
            // 2:10 remaining
            if (time <= 130.0) {
                return false;
            }
            // 2:20 remaining
            if (time <= 140) {
                return true;
            }
            return false;
        } else if (state == HubState.NODATA) {
            return true;
        } else {
            return true;
        }
    }

    private static enum HubState {
        FIRST, SECOND, NODATA
    }

    public static boolean currentHubIsActive() {
        return checkHub(activeHubAfterAuto());
    }
}
