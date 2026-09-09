package frc.robot.ballistics;

public class RebuiltBallConstants {

    public static final double GRAVITY = 9.81; // m/s^2
    public static final double AIR_DENSITY = 1.225; // kg/m^3, standard atmosphere
    public static final double BALL_MASS = 0.215; // kg, game manual 5.10.1 midpoint
    public static final double BALL_DIAMETER = 0.1501; // m, game manual 5.10.1
    public static final double BALL_RADIUS = BALL_DIAMETER / 2.0;
    public static final double BALL_CROSS_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
    public static final double CD = 0.47; // drag coefficient, smooth sphere

}
