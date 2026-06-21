package frc.robot.shotdata;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import java.io.IOException;
import java.util.Optional;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.math.interp2d.BilinearMap;
import frc.robot.math.interp2d.MulAdd;
import frc.robot.util.binrw.BinaryData;
import frc.robot.util.binrw.Binrw;
import frc.robot.util.tunable.Tunable;

/**
 * Loads the solver-generated shot table ({@code shots-rt.bin}) and interpolates it.
 *
 * <p>
 * The table is keyed by target distance and radial velocity. A lookup returns the flywheel speed,
 * hood angle, and time of flight, bilinearly interpolated between the surrounding entries.
 */
@Tunable
public class ShotData {

    public double linearParameter = 9.7;
    public static final double backspinParameter = 0.1;

    /** Set of shot entries. */
    @Binrw
    public static record ShotEntrySet(ShotEntry[] entries) {
    }

    /**
     * One table entry: the solved shooter parameters for a single (target distance, radial
     * velocity) pair.
     */
    @Binrw
    public static record ShotEntry(Distance targetDistance, LinearVelocity radialVelocity,
        Angle exitAngle, Angle minAngle, Angle maxAngle, LinearVelocity exitVelocity,
        LinearVelocity minVelocity, LinearVelocity maxVelocity, Time tof) {
    }

    /** Setpoints for a given shot */
    public static record ShotParameters(double hoodAngle, double minHoodAngle, double maxHoodAngle,
        double flywheelSpeedRps, double minFlywheelSpeed, double maxFlywheelSpeed, double tof) {
    }

    /**
     * Addition and scalar multiplication over {@link ShotEntry} objects, used by the bilinear map
     * to interpolate between entries.
     */
    public static final MulAdd<ShotEntry> mulAdd = new MulAdd<ShotEntry>() {

        @Override
        public ShotEntry mul(ShotEntry a, double b) {
            return new ShotEntry(a.targetDistance.times(b), a.radialVelocity.times(b),
                a.exitAngle.times(b), a.minAngle.times(b), a.maxAngle.times(b),
                a.exitVelocity.times(b), a.minVelocity.times(b), a.maxVelocity.times(b),
                a.tof.times(b));
        }

        @Override
        public ShotEntry add(ShotEntry a, ShotEntry b) {
            return new ShotEntry(a.targetDistance.plus(b.targetDistance),
                a.radialVelocity.plus(b.radialVelocity), a.exitAngle.plus(b.exitAngle),
                a.minAngle.plus(b.minAngle), a.maxAngle.plus(b.maxAngle),
                a.exitVelocity.plus(b.exitVelocity), a.minVelocity.plus(b.minVelocity),
                a.maxVelocity.plus(b.maxVelocity), a.tof.plus(b.tof));
        }

    };

    private static final BilinearMap<ShotEntry> shotEntries = new BilinearMap<>(mulAdd);

    static {
        try {
            var res = BinaryData.readFile("shots-rt.bin", ShotEntrySetReader::read);
            for (var entry : res.entries()) {
                shotEntries.put(entry.targetDistance.in(Meters),
                    entry.radialVelocity.in(MetersPerSecond), entry);
            }
        } catch (IOException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    /** Get shot parameters for a given distance and radial velocity. */
    public Optional<ShotParameters> getShotEntry(Distance distance, LinearVelocity radialVelocity) {
        var entry = shotEntries.get(distance.in(Meters), radialVelocity.in(MetersPerSecond));
        return entry.map(entry_ -> new ShotParameters(hoodAngle(entry_.exitAngle),
            hoodAngle(entry_.minAngle), hoodAngle(entry_.maxAngle),
            linearParameter * entry_.exitVelocity.in(MetersPerSecond),
            linearParameter * entry_.minVelocity.in(MetersPerSecond),
            linearParameter * entry_.maxVelocity.in(MetersPerSecond), entry_.tof().in(Seconds)));
    }

    private static double hoodAngle(Angle exitAngle) {
        return 90 - exitAngle.in(Degrees) - 12.985;
    }

}
