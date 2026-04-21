package frc.gen;

import java.util.function.Function;
import java.util.stream.IntStream;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Constraints;
import org.wpilib.math.optimization.Problem;

public class ShooterBallistics {

    private static final double ballMass = 0.215; // kg
    private static final double ballDiameter = 0.15; // m
    private static final double airDensity = 1.18; // kg/m^3
    private static final double dragCoeff = 0.63;
    private static final double magnusCoeff = 0.2;
    private static final double gravity = 9.81; // m/s^2
    private static final int numSamples = 20;

    private static final double minHoodAngle = 12.965;
    private static final double maxHoodAngle = 42.0;

    public final Variable tof;
    public final Variable backspin;
    public final Variable exitAngle;
    public final Variable exitVelocity;
    private final Variable dt;

    public final Variable[] xs;
    public final Variable[] zs;
    public final Variable[] vxs;
    public final Variable[] vzs;

    public ShooterBallistics(Problem problem, double shooterHeight, double targetHeight,
        double targetDistance) {
        var ballRadius = ballDiameter / 2.0;
        var magnusFactor = 0.5 * Math.pow(ballRadius, 3) * magnusCoeff * Math.PI * airDensity;
        var dragFactor = -Math.pow(ballRadius, 2) * dragCoeff * Math.PI * airDensity;

        // 2. Decision Variables
        backspin = problem.decisionVariable();
        problem.subjectTo(Constraints.le(backspin, 0.0));
        problem.subjectTo(Constraints.ge(backspin, -7.0));

        // Time of flight (Constraint: must be positive)
        tof = problem.decisionVariable();
        problem.subjectTo(Constraints.gt(tof, 0));
        tof.setValue(1.0);

        dt = tof.div(numSamples);

        var X = problem.decisionVariable(4, numSamples);
        xs = IntStream.range(0, numSamples).mapToObj(i -> X.get(0, i)).toArray(Variable[]::new);
        zs = IntStream.range(0, numSamples).mapToObj(i -> X.get(1, i)).toArray(Variable[]::new);
        vxs = IntStream.range(0, numSamples).mapToObj(i -> X.get(2, i)).toArray(Variable[]::new);
        vzs = IntStream.range(0, numSamples).mapToObj(i -> X.get(3, i)).toArray(Variable[]::new);

        exitAngle = Variable.atan2(zs[0], xs[0]);
        problem.subjectTo(Constraints.ge(exitAngle, Math.toRadians(90 - maxHoodAngle)));
        problem.subjectTo(Constraints.le(exitAngle, Math.toRadians(90 - minHoodAngle)));
        exitVelocity = Variable.hypot(zs[0], xs[0]);

        // 3. Dynamics Helper
        Function<VariableMatrix, VariableMatrix> calculateDynamics = (state) -> {
            var vx = state.get(2);
            var vz = state.get(3);

            var vMag = Variable.hypot(vx, vz);

            var magnusX = vz.times(backspin).times(magnusFactor);
            var magnusZ = vx.times(backspin).times(-magnusFactor);

            var dragX = vx.times(vMag).times(dragFactor);
            var dragZ = vz.times(vMag).times(dragFactor);

            return new VariableMatrix(new Variable[][] {new Variable[] {vx}, new Variable[] {vz},
                new Variable[] {magnusX.plus(dragX).div(ballMass)},
                new Variable[] {magnusZ.plus(dragZ).minus(ballMass * gravity).div(ballMass)}});
        };

        // 4. Initial Guesses
        for (int k = 0; k < numSamples; k++) {
            double ratio = ((double) k) / numSamples;

            xs[k].setValue(ratio * targetDistance);
            zs[k].setValue(ratio * (targetHeight - shooterHeight) + shooterHeight);
            vxs[k].setValue(targetDistance / numSamples);
            vzs[k].setValue((targetHeight - shooterHeight) / numSamples);
        }

        // 5. Constraints
        problem.subjectTo(Constraints.eq(X.get(0, 0), 0.0));
        problem.subjectTo(Constraints.eq(X.get(1, 0), shooterHeight));
        problem.subjectTo(Constraints.eq(X.get(0, numSamples - 1), targetDistance));
        problem.subjectTo(Constraints.eq(X.get(1, numSamples - 1), targetHeight));

        // 6. Integration (RK4)
        for (int k = 0; k < numSamples - 1; k++) {
            var x_k = X.col(k);
            var x_k1 = X.col(k + 1);
            var h = dt;

            var k1 = calculateDynamics.apply(new VariableMatrix(x_k));
            var k2 = calculateDynamics.apply(x_k.plus(k1.times(h).div(2)));
            var k3 = calculateDynamics.apply(x_k.plus(k2.times(h).div(2)));
            var k4 = calculateDynamics.apply(x_k.plus(k3.times(h)));

            problem.subjectTo(Constraints.eq(x_k1,
                x_k.plus((k1.plus(k2.times(2)).plus(k3.times(2)).plus(k4)).times(h).div(6))));
        }
    }

    @Override
    public String toString() {
        var sb = new StringBuilder("ShooterBallistics {");
        sb.append("tof=");
        sb.append(tof.value());
        sb.append(",backspin=");
        sb.append(backspin.value());
        sb.append(",exitAngle=");
        sb.append(Math.toDegrees(exitAngle.value()));
        sb.append(",exitVelocity=");
        sb.append(exitVelocity.value());
        sb.append("}");
        return sb.toString();
    }

}
