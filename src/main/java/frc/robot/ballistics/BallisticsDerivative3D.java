package frc.robot.ballistics;

public class BallisticsDerivative3D implements DerivativeFunction {

    public final double density;
    public final double dragCoefficient;
    public final double crossSectionalArea;
    public final double gravity;
    public final double mass;

    public BallisticsDerivative3D(double density, double dragCoefficient, double crossSectionalArea,
        double gravity, double mass) {
        this.density = density;
        this.dragCoefficient = dragCoefficient;
        this.crossSectionalArea = crossSectionalArea;
        this.gravity = gravity;
        this.mass = mass;
    }

    public BallisticsDerivative3D() {
        this(RebuiltBallConstants.AIR_DENSITY, RebuiltBallConstants.CD,
            RebuiltBallConstants.BALL_CROSS_AREA, RebuiltBallConstants.GRAVITY,
            RebuiltBallConstants.BALL_MASS);
    }

    @Override
    public double[] derivative(double[] input) {
        @SuppressWarnings("unused")
        double px = input[0];
        @SuppressWarnings("unused")
        double py = input[1];
        @SuppressWarnings("unused")
        double pz = input[2];
        double vx = input[3];
        double vy = input[4];
        double vz = input[5];

        double speed = Math.hypot(Math.hypot(vx, vy), vz);
        double dragMagnitude = speed * speed * 0.5 * density * dragCoefficient * crossSectionalArea;

        double ax = 0.0;
        double ay = 0.0;
        double az = -gravity;

        // At 0 speed, no drag
        if (speed > 1e-6) {
            ax += -dragMagnitude * vx / (speed * mass);
            ay += -dragMagnitude * vy / (speed * mass);
            az += -dragMagnitude * vz / (speed * mass);
        }

        return new double[] {vx, vy, vz, ax, ay, az};
    }



}
