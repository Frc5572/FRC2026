package frc.gen;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.FieldConstants;
import frc.robot.shotdata.ShotData;
import frc.robot.shotdata.SimulatedShot;

/** Info regarding a simulated trajectory */
public class TrajectoryInfo {

    public final Angle exitAngle;
    public final LinearVelocity exitVelocity;
    public final AngularVelocity backspin;
    public final Distance apogeeHeight;
    public final Distance apogeeDistance;
    public final Distance clearanceOverLip;
    public final Distance hubDistance;
    public final Distance groundDistance;
    public final Time tofHub;
    public final Time tofGround;
    public final boolean reachesHub;

    private static final double dt = 0.001;

    /** Simulate and aggregate info for trajectory */
    public TrajectoryInfo(Angle exitAngle, LinearVelocity exitVelocity, AngularVelocity backspin) {
        SimulatedShot shot = new SimulatedShot(exitAngle, exitVelocity, backspin);
        InterpolatingDoubleTreeMap past = new InterpolatingDoubleTreeMap();
        double t = 0.0;
        boolean hasApogee = false;
        boolean hasHub = false;
        Translation2d apogee = Translation2d.kZero;
        double z = ShotData.shooterToTargetHeightDiff.in(Meters);

        double hubDistance = 0.0;
        boolean reachesHub = false;
        double tofHub = 0.0;

        while (shot.state.a2 >= 0.0) {
            shot.step(dt);
            t += dt;
            past.put(shot.state.a1, shot.state.a2);
            if (shot.state.a4 < 0.0 && !hasApogee) {
                apogee = new Translation2d(shot.state.a1, shot.state.a2);
                hasApogee = true;
                if (shot.state.a2 > z) {
                    reachesHub = true;
                }
            }
            if (reachesHub && shot.state.a2 < z && !hasHub) {
                hubDistance = shot.state.a1;
                tofHub = t;
                hasHub = true;
            }
        }
        this.apogeeHeight = Meters.of(apogee.getY());
        this.apogeeDistance = Meters.of(apogee.getX());
        this.clearanceOverLip =
            Meters.of(past.get(hubDistance - FieldConstants.Hub.width / 2.0) - z);
        this.hubDistance = Meters.of(hubDistance);
        this.groundDistance = Meters.of(shot.state.a1);
        this.reachesHub = reachesHub;
        this.tofHub = Seconds.of(tofHub);
        this.tofGround = Seconds.of(t);
        this.exitAngle = exitAngle;
        this.exitVelocity = exitVelocity;
        this.backspin = backspin;
    }

}
