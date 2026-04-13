package frc.robot.shotdata;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

/**
 * Auto-generated lookup tables (LUTs) and fitted model coefficients for shooter calculations.
 *
 * <p>
 * This class is not meant to be instantiated or modified manually - values arederived from
 * experimental shot data and curve-fitting.
 */
public final class GeneratedLUTs2 {
    /**
     * Pre-computed shot entries for hub-targeted shots.
     */
    public static final ShotData.ShotEntry[] hubEntries = new ShotData.ShotEntry[] {
        new ShotData.ShotEntry(Meters.of(2.198), RotationsPerSecond.of(45.000), Degrees.of(59.305),
            MetersPerSecond.of(6.671), Seconds.of(0.710)),
        new ShotData.ShotEntry(Meters.of(2.438), RotationsPerSecond.of(46.000), Degrees.of(57.305),
            MetersPerSecond.of(6.892), Seconds.of(0.726)),
        new ShotData.ShotEntry(Meters.of(2.758), RotationsPerSecond.of(50.000), Degrees.of(57.305),
            MetersPerSecond.of(7.181), Seconds.of(0.798)),
        new ShotData.ShotEntry(Meters.of(3.088), RotationsPerSecond.of(52.000), Degrees.of(57.305),
            MetersPerSecond.of(7.500), Seconds.of(0.868)),
        new ShotData.ShotEntry(Meters.of(3.383), RotationsPerSecond.of(54.000), Degrees.of(57.305),
            MetersPerSecond.of(7.799), Seconds.of(0.926)),
        new ShotData.ShotEntry(Meters.of(3.673), RotationsPerSecond.of(56.000), Degrees.of(57.305),
            MetersPerSecond.of(8.097), Seconds.of(0.981)),
        new ShotData.ShotEntry(Meters.of(3.908), RotationsPerSecond.of(58.000), Degrees.of(57.305),
            MetersPerSecond.of(8.339), Seconds.of(1.024)),
        new ShotData.ShotEntry(Meters.of(4.228), RotationsPerSecond.of(61.000), Degrees.of(56.305),
            MetersPerSecond.of(8.635), Seconds.of(1.053)),
        new ShotData.ShotEntry(Meters.of(4.575), RotationsPerSecond.of(64.000), Degrees.of(55.305),
            MetersPerSecond.of(8.955), Seconds.of(1.085)),
        new ShotData.ShotEntry(Meters.of(4.919), RotationsPerSecond.of(67.000), Degrees.of(54.305),
            MetersPerSecond.of(9.271), Seconds.of(1.113)),
        new ShotData.ShotEntry(Meters.of(4.992), RotationsPerSecond.of(67.000), Degrees.of(52.305),
            MetersPerSecond.of(9.295), Seconds.of(1.074)),
        new ShotData.ShotEntry(Meters.of(5.212), RotationsPerSecond.of(69.000), Degrees.of(52.305),
            MetersPerSecond.of(9.512), Seconds.of(1.106)),
        new ShotData.ShotEntry(Meters.of(5.423), RotationsPerSecond.of(71.000), Degrees.of(52.305),
            MetersPerSecond.of(9.724), Seconds.of(1.136)),
        new ShotData.ShotEntry(Meters.of(5.635), RotationsPerSecond.of(73.000), Degrees.of(52.305),
            MetersPerSecond.of(9.934), Seconds.of(1.166)),
        new ShotData.ShotEntry(Meters.of(5.838), RotationsPerSecond.of(75.000), Degrees.of(52.305),
            MetersPerSecond.of(10.139), Seconds.of(1.194)),
        new ShotData.ShotEntry(Meters.of(6.036), RotationsPerSecond.of(77.000), Degrees.of(52.305),
            MetersPerSecond.of(10.342), Seconds.of(1.221)),
        new ShotData.ShotEntry(Meters.of(6.230), RotationsPerSecond.of(79.000), Degrees.of(52.305),
            MetersPerSecond.of(10.540), Seconds.of(1.247)),
        new ShotData.ShotEntry(Meters.of(6.419), RotationsPerSecond.of(81.000), Degrees.of(52.305),
            MetersPerSecond.of(10.735), Seconds.of(1.272)),
        new ShotData.ShotEntry(Meters.of(6.602), RotationsPerSecond.of(83.000), Degrees.of(52.305),
            MetersPerSecond.of(10.926), Seconds.of(1.296)),
        new ShotData.ShotEntry(Meters.of(6.780), RotationsPerSecond.of(85.000), Degrees.of(52.305),
            MetersPerSecond.of(11.114), Seconds.of(1.319)),
        new ShotData.ShotEntry(Meters.of(6.957), RotationsPerSecond.of(87.000), Degrees.of(52.305),
            MetersPerSecond.of(11.298), Seconds.of(1.342)),
        new ShotData.ShotEntry(Meters.of(7.125), RotationsPerSecond.of(89.000), Degrees.of(52.305),
            MetersPerSecond.of(11.478), Seconds.of(1.363))};

    /**
     * Pre-computed shot entries for passing.
     */
    public static final ShotData.ShotEntry[] groundEntries = new ShotData.ShotEntry[] {
        new ShotData.ShotEntry(Meters.of(3.238), RotationsPerSecond.of(45.000), Degrees.of(59.305),
            MetersPerSecond.of(6.671), Seconds.of(1.093)),
        new ShotData.ShotEntry(Meters.of(3.534), RotationsPerSecond.of(46.000), Degrees.of(57.305),
            MetersPerSecond.of(6.892), Seconds.of(1.101)),
        new ShotData.ShotEntry(Meters.of(3.775), RotationsPerSecond.of(50.000), Degrees.of(57.305),
            MetersPerSecond.of(7.181), Seconds.of(1.141)),
        new ShotData.ShotEntry(Meters.of(4.045), RotationsPerSecond.of(52.000), Degrees.of(57.305),
            MetersPerSecond.of(7.500), Seconds.of(1.185)),
        new ShotData.ShotEntry(Meters.of(4.297), RotationsPerSecond.of(54.000), Degrees.of(57.305),
            MetersPerSecond.of(7.799), Seconds.of(1.225)),
        new ShotData.ShotEntry(Meters.of(4.553), RotationsPerSecond.of(56.000), Degrees.of(57.305),
            MetersPerSecond.of(8.097), Seconds.of(1.265)),
        new ShotData.ShotEntry(Meters.of(4.759), RotationsPerSecond.of(58.000), Degrees.of(57.305),
            MetersPerSecond.of(8.339), Seconds.of(1.296)),
        new ShotData.ShotEntry(Meters.of(5.092), RotationsPerSecond.of(61.000), Degrees.of(56.305),
            MetersPerSecond.of(8.635), Seconds.of(1.320)),
        new ShotData.ShotEntry(Meters.of(5.446), RotationsPerSecond.of(64.000), Degrees.of(55.305),
            MetersPerSecond.of(8.955), Seconds.of(1.344)),
        new ShotData.ShotEntry(Meters.of(5.803), RotationsPerSecond.of(67.000), Degrees.of(54.305),
            MetersPerSecond.of(9.271), Seconds.of(1.367)),
        new ShotData.ShotEntry(Meters.of(5.953), RotationsPerSecond.of(67.000), Degrees.of(52.305),
            MetersPerSecond.of(9.295), Seconds.of(1.335)),
        new ShotData.ShotEntry(Meters.of(6.155), RotationsPerSecond.of(69.000), Degrees.of(52.305),
            MetersPerSecond.of(9.512), Seconds.of(1.361)),
        new ShotData.ShotEntry(Meters.of(6.348), RotationsPerSecond.of(71.000), Degrees.of(52.305),
            MetersPerSecond.of(9.724), Seconds.of(1.385)),
        new ShotData.ShotEntry(Meters.of(6.541), RotationsPerSecond.of(73.000), Degrees.of(52.305),
            MetersPerSecond.of(9.934), Seconds.of(1.409)),
        new ShotData.ShotEntry(Meters.of(6.728), RotationsPerSecond.of(75.000), Degrees.of(52.305),
            MetersPerSecond.of(10.139), Seconds.of(1.432)),
        new ShotData.ShotEntry(Meters.of(6.910), RotationsPerSecond.of(77.000), Degrees.of(52.305),
            MetersPerSecond.of(10.342), Seconds.of(1.454)),
        new ShotData.ShotEntry(Meters.of(7.091), RotationsPerSecond.of(79.000), Degrees.of(52.305),
            MetersPerSecond.of(10.540), Seconds.of(1.476)),
        new ShotData.ShotEntry(Meters.of(7.266), RotationsPerSecond.of(81.000), Degrees.of(52.305),
            MetersPerSecond.of(10.735), Seconds.of(1.497)),
        new ShotData.ShotEntry(Meters.of(7.440), RotationsPerSecond.of(83.000), Degrees.of(52.305),
            MetersPerSecond.of(10.926), Seconds.of(1.518)),
        new ShotData.ShotEntry(Meters.of(7.608), RotationsPerSecond.of(85.000), Degrees.of(52.305),
            MetersPerSecond.of(11.114), Seconds.of(1.538)),
        new ShotData.ShotEntry(Meters.of(7.771), RotationsPerSecond.of(87.000), Degrees.of(52.305),
            MetersPerSecond.of(11.298), Seconds.of(1.557)),
        new ShotData.ShotEntry(Meters.of(7.932), RotationsPerSecond.of(89.000), Degrees.of(52.305),
            MetersPerSecond.of(11.478), Seconds.of(1.576)),
        new ShotData.ShotEntry(Meters.of(8.022), RotationsPerSecond.of(85.000), Degrees.of(42.305),
            MetersPerSecond.of(11.114), Seconds.of(1.321)),
        new ShotData.ShotEntry(Meters.of(8.200), RotationsPerSecond.of(87.000), Degrees.of(42.305),
            MetersPerSecond.of(11.298), Seconds.of(1.338)),
        new ShotData.ShotEntry(Meters.of(8.371), RotationsPerSecond.of(89.000), Degrees.of(42.305),
            MetersPerSecond.of(11.478), Seconds.of(1.354))};
}
