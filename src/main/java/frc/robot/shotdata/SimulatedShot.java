package frc.robot.shotdata;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import org.ejml.data.DMatrix2;
import org.ejml.data.DMatrix4;
import org.ejml.dense.fixed.CommonOps_DDF2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class SimulatedShot {

    public final Angle exitAngle;
    public final LinearVelocity exitVelocity;
    public final AngularVelocity backspin;

    private static final double g = 9.81; // m/s^2
    private static final double rho = 1.225; // kg/m^3, standard atmosphere
    private static final double m = 0.215; // kg, game manual 5.10.1 midpoint
    private static final double r = 0.1501 / 2.0; // m, game manual 5.10.1

    // Aerodynamic coefficients
    private static final double C_D = 0.63; // drag coefficient, smooth sphere
    private static final double C_L = 0.2; // Magnus coefficient, conservative estimate

    public final double omega;
    public final DMatrix4 state = new DMatrix4();
    private final DMatrix2 forces = new DMatrix2();

    private static final double MAGNUS_FACTOR = 0.5 * Math.pow(r, 3) * C_L * Math.PI * rho;
    private static final double DRAG_FACTOR = -Math.pow(r, 2) * C_D * Math.PI * rho;

    private void updateForces() {
        double vx = state.a3;
        double vz = state.a4;

        double vmag = Math.hypot(vx, vz);

        double magnus_x = MAGNUS_FACTOR * vz * omega;
        double magnus_z = -MAGNUS_FACTOR * vx * omega;

        double drag_x = DRAG_FACTOR * vx * vmag;
        double drag_z = DRAG_FACTOR * vz * vmag;

        forces.a1 = magnus_x + drag_x;
        forces.a2 = magnus_z + drag_z - m * g;
    }

    public SimulatedShot(Angle exitAngle, LinearVelocity exitVelocity, AngularVelocity backspin) {
        this.exitAngle = exitAngle;
        this.exitVelocity = exitVelocity;
        this.backspin = backspin;
        state.a1 = 0;
        state.a2 = 0;
        state.a3 = exitVelocity.in(MetersPerSecond) * Math.cos(exitAngle.in(Radians));
        state.a4 = exitVelocity.in(MetersPerSecond) * Math.sin(exitAngle.in(Radians));
        this.omega = backspin.in(RadiansPerSecond);
    }

    public void step(double dt) {
        updateForces();
        CommonOps_DDF2.scale(1.0 / m, forces);
        state.a1 += dt * state.a3;
        state.a2 += dt * state.a4;
        state.a3 += dt * forces.a1;
        state.a4 += dt * forces.a2;
    }

}
