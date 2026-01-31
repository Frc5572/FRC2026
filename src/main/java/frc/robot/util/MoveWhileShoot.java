package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants.Hub;

public class MoveWhileShoot {
    private static final InterpolatingTreeMap<Double, Double> shooterMap =
        new InterpolatingTreeMap<>(null, null);
    static {
        shooterMap.put(1.5, 2800.0);
        shooterMap.put(2.0, 3100.0);
        shooterMap.put(2.5, 3400.0);
        shooterMap.put(3.0, 3650.0);
        shooterMap.put(3.5, 3900.0);
        shooterMap.put(4.0, 4100.0);
        shooterMap.put(4.5, 4350.0);
        shooterMap.put(5.0, 4550.0);
    }



    public class SuperShooter {
        public void update(Pose2d robotPose, ChassisSpeeds robotSpeed) {


            double targetX = Hub.topCenterPoint.toTranslation2d().getX() - robotPose.getX();
            double targetY = Hub.topCenterPoint.toTranslation2d().getY() - robotPose.getY();
            Translation2d targetPosition = new Translation2d(targetX, targetY); // position
                                                                                // resultant from
                                                                                // previous math
            double distance = targetPosition.getNorm(); // dist from bot to hub

        }
    }
}
