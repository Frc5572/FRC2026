package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        if (state == HubState.FIRST) {
            var time = Timer.getFPGATimestamp();
            if ((time <= 140.0) && (time >= 130.0)) {
                return true;
            } else if ((time <= 129.0) && (time >= 105.0)) {
                return false;
            } else if ((time <= 104.0) && (time >= 55.0)) {
                return true;
            } else if (time <= 54.0) {
                return true;
            } else {
                return false;
            }
        } else if (state == HubState.SECOND) {
            var time = Timer.getFPGATimestamp();
            if ((time <= 140.0) && (time >= 130.0)) {
                return true;
            } else if ((time <= 129.0) && (time >= 105.0)) {
                return true;
            } else if ((time <= 104.0) && (time >= 55.0)) {
                return false;
            } else if (time <= 54.0) {
                return true;
            } else {
                return false;
            }
        } else if (state == HubState.NODATA) {
            return true;
        } else {
            return true;
        }
    }

    private static enum HubState {
        FIRST, SECOND, NODATA
    }
}
