package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ActiveHub {

    public static Trigger activeHub = new Trigger(() -> checkHub(activeHubAfterAuto()));

    public ActiveHub() {}

    private static hubState activeHubAfterAuto() {
        var gamedata = DriverStation.getGameSpecificMessage();
        if (gamedata.length() > 0) {
            switch (gamedata.charAt(0)) {
                case 'B':
                    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
                        return hubState.active;
                    } else {
                        return hubState.deactivated;
                    }
                case 'R':
                    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
                        return hubState.active;
                    } else {
                        return hubState.deactivated;
                    }
                default:
                    return hubState.active;
            }
        } else {
            return hubState.no_data;
        }
    }

    private static boolean checkHub(hubState state) {
        if (state == hubState.active) {
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
        } else if (state == hubState.deactivated) {
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
        } else if (state == hubState.no_data) {
            return true;
        } else {
            return true;
        }
    }

    private static enum hubState {
        active, deactivated, no_data
    }
}
