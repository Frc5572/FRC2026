package frc.robot.math.geometry;

import edu.wpi.first.math.geometry.Translation2d;

/** Represents a convex shape for the purpose of separating axis solving. */
public interface ConvexShape {

    /** Get the {@link Axis axes} of this {@link ConvexShape}. */
    public Axis[] getAxes();

    /**
     * Get the {@link Interval} of this {@link ConvexShape} projected onto the given {@link Axis}.
     */
    public Interval project(Axis axis);

    /**
     * Get the center of this {@link ConvexShape}.
     */
    public Translation2d getCenter();

}
