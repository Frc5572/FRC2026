package frc.gen;

import java.util.stream.IntStream;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.optimization.Constraints;
import org.wpilib.math.optimization.Problem;

public class SleipnirTrajectory {

    private final Variable omega;
    public final Variable dt;

    public final Variable[] finalState;

    public SleipnirTrajectory(Problem problem, Variable exitAngle, Variable exitVelocity,
        Variable backspin, int numSections) {
        this.omega = backspin;
        this.dt = problem.decisionVariable();
        problem.subjectTo(Constraints.gt(dt, 0.0));
        var vx = Variable.cos(exitAngle).times(exitVelocity);
        var vz = Variable.sin(exitAngle).times(exitVelocity);

        var x = vx.times(0.0001);
        var z = vz.times(0.0001);

        var state = new Variable[] {x, z, vx, vz};

        for (int i = 0; i < numSections; i++) {
            rk4(state);
        }

        this.finalState = state;
    }

    private static final double g = 9.81; // m/s^2
    private static final double rho = 1.225; // kg/m^3, standard atmosphere
    private static final double m = 0.215; // kg, game manual 5.10.1 midpoint
    private static final double r = 0.1501 / 2.0; // m, game manual 5.10.1

    // Aerodynamic coefficients
    private static final double C_D = 0.47; // drag coefficient, smooth sphere
    private static final double C_L = 0.2; // Magnus coefficient, conservative estimate
    private static final double MAGNUS_FACTOR = 0.5 * Math.pow(r, 3) * C_L * Math.PI * rho;
    private static final double DRAG_FACTOR = -Math.pow(r, 2) * C_D * Math.PI * rho;

    private Variable[] derivative(Variable[] state) {
        var vx = state[2];
        var vz = state[3];
        var vmag = Variable.hypot(vx, vz);

        var magnus_x = vz.times(MAGNUS_FACTOR).times(omega);
        var magnus_z = vx.times(-MAGNUS_FACTOR).times(omega);

        var drag_x = vx.times(vmag).times(DRAG_FACTOR);
        var drag_z = vz.times(vmag).times(DRAG_FACTOR);

        var fx = magnus_x.plus(drag_x);
        var fz = magnus_z.plus(drag_z).minus(m * g);

        return new Variable[] {vx, vz, fx.div(m), fz.div(m)};
    }

    private void rk4(Variable[] state) {
        var k1 = derivative(state);
        var testPoint1 = IntStream.range(0, 4).mapToObj(i -> state[i].plus(k1[i].times(dt.div(2))))
            .toArray(Variable[]::new);
        var k2 = derivative(testPoint1);
        var testPoint2 = IntStream.range(0, 4).mapToObj(i -> state[i].plus(k2[i].times(dt.div(2))))
            .toArray(Variable[]::new);
        var k3 = derivative(testPoint2);
        var testPoint3 = IntStream.range(0, 4).mapToObj(i -> state[i].plus(k3[i].times(dt)))
            .toArray(Variable[]::new);
        var k4 = derivative(testPoint3);

        for (int i = 0; i < 4; i++) {
            state[i] = state[i]
                .plus(dt.div(6).times(k1[i].plus(k2[i].times(2)).plus(k3[i].times(2)).plus(k4[i])));
        }
    }

}
