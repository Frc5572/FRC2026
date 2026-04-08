package frc.robot.shotdata;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class ProjectileTrajectory {

    public final Angle exitAngle;
    public final LinearVelocity exitVelocity;

    public ProjectileTrajectory(Angle exitAngle, LinearVelocity exitVelocity) {
        this.exitAngle = exitAngle;
        this.exitVelocity = exitVelocity;
    }

    public ProjectileTrajectory(Angle exitAngle, Translation2d pointOnTrajectory) {
        this(exitAngle, v0(exitAngle, pointOnTrajectory));
    }

    public double x(double t) {
        return exitVelocity.in(MetersPerSecond) * t * Math.cos(exitAngle.in(Radians));
    }

    public double y(double t) {
        return exitVelocity.in(MetersPerSecond) * t * Math.sin(exitAngle.in(Radians))
            - 9.81 * 0.5 * t * t;
    }

    private static LinearVelocity v0(Angle angle, Translation2d pointOnTrajectory) {
        double x = pointOnTrajectory.getX();
        double y = pointOnTrajectory.getY();
        double theta = angle.in(Radians);
        double g = 9.81;

        return MetersPerSecond.of(Math.sqrt(
            (x * x * g) / (x * Math.sin(2 * theta) - 2 * y * Math.cos(theta) * Math.cos(theta))));
    }

}
