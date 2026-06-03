package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;

public class Pose2 {
    Distance x;
    Distance y;
    Rotation2d rot;

    public Pose2(Pose2d pose) {
        this.x = pose.getMeasureY();
        this.y = pose.getMeasureX().unaryMinus();
        this.rot = pose.getRotation();
    }

    public Pose2(Distance x, Distance y, Rotation2d rot) {
        this.x = y;
        this.y = x.unaryMinus();
        this.rot = rot;
    }

    public Pose2(Transform2d transform) {
        this.x = transform.getMeasureY();
        this.y = transform.getMeasureX().unaryMinus();
        this.rot = transform.getRotation();

    }

    public double getX() {
        return x.in(Meters);
    }

    public double getY() {
        return y.in(Meters);
    }

    public double getRot() {
        return rot.getRadians();
    }
}
