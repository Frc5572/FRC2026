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
        new ShotData.ShotEntry(Meters.of(2.146), RotationsPerSecond.of(43.000), Degrees.of(62.305),
            MetersPerSecond.of(6.649), Seconds.of(0.765)),
        new ShotData.ShotEntry(Meters.of(2.438), RotationsPerSecond.of(44.000), Degrees.of(62.305),
            MetersPerSecond.of(6.959), Seconds.of(0.841)),
        new ShotData.ShotEntry(Meters.of(2.798), RotationsPerSecond.of(45.000), Degrees.of(62.305),
            MetersPerSecond.of(7.363), Seconds.of(0.928)),
        new ShotData.ShotEntry(Meters.of(3.231), RotationsPerSecond.of(48.000), Degrees.of(57.305),
            MetersPerSecond.of(7.643), Seconds.of(0.896)),
        new ShotData.ShotEntry(Meters.of(3.798), RotationsPerSecond.of(53.000), Degrees.of(57.305),
            MetersPerSecond.of(8.226), Seconds.of(1.003)),
        new ShotData.ShotEntry(Meters.of(4.167), RotationsPerSecond.of(54.000), Degrees.of(55.305),
            MetersPerSecond.of(8.539), Seconds.of(1.018)),
        new ShotData.ShotEntry(Meters.of(4.404), RotationsPerSecond.of(56.000), Degrees.of(53.305),
            MetersPerSecond.of(8.732), Seconds.of(1.008)),
        new ShotData.ShotEntry(Meters.of(4.801), RotationsPerSecond.of(57.000), Degrees.of(52.805),
            MetersPerSecond.of(9.113), Seconds.of(1.058)),
        new ShotData.ShotEntry(Meters.of(5.026), RotationsPerSecond.of(57.500), Degrees.of(49.305),
            MetersPerSecond.of(9.313), Seconds.of(1.006)),
        new ShotData.ShotEntry(Meters.of(5.532), RotationsPerSecond.of(61.000), Degrees.of(49.305),
            MetersPerSecond.of(9.795), Seconds.of(1.076)),
        new ShotData.ShotEntry(Meters.of(5.822), RotationsPerSecond.of(63.000), Degrees.of(49.305),
            MetersPerSecond.of(10.074), Seconds.of(1.114)),
        new ShotData.ShotEntry(Meters.of(6.157), RotationsPerSecond.of(65.000), Degrees.of(49.305),
            MetersPerSecond.of(10.399), Seconds.of(1.157)),
        new ShotData.ShotEntry(Meters.of(6.492), RotationsPerSecond.of(67.000), Degrees.of(49.305),
            MetersPerSecond.of(10.730), Seconds.of(1.199)),
        new ShotData.ShotEntry(Meters.of(6.858), RotationsPerSecond.of(69.000), Degrees.of(49.305),
            MetersPerSecond.of(11.097), Seconds.of(1.245)),
        new ShotData.ShotEntry(Meters.of(7.100), RotationsPerSecond.of(71.000), Degrees.of(52.305),
            MetersPerSecond.of(11.451), Seconds.of(1.360)),
        new ShotData.ShotEntry(Meters.of(7.432), RotationsPerSecond.of(73.000), Degrees.of(52.305),
            MetersPerSecond.of(11.803), Seconds.of(1.402)),
        new ShotData.ShotEntry(Meters.of(7.762), RotationsPerSecond.of(75.000), Degrees.of(52.305),
            MetersPerSecond.of(12.159), Seconds.of(1.443)),
        new ShotData.ShotEntry(Meters.of(8.089), RotationsPerSecond.of(77.000), Degrees.of(52.305),
            MetersPerSecond.of(12.517), Seconds.of(1.483)),
        new ShotData.ShotEntry(Meters.of(8.413), RotationsPerSecond.of(79.000), Degrees.of(52.305),
            MetersPerSecond.of(12.878), Seconds.of(1.522)),
        new ShotData.ShotEntry(Meters.of(8.734), RotationsPerSecond.of(81.000), Degrees.of(52.305),
            MetersPerSecond.of(13.242), Seconds.of(1.560)),
        new ShotData.ShotEntry(Meters.of(9.056), RotationsPerSecond.of(83.000), Degrees.of(52.305),
            MetersPerSecond.of(13.608), Seconds.of(1.598)),
        new ShotData.ShotEntry(Meters.of(9.374), RotationsPerSecond.of(85.000), Degrees.of(52.305),
            MetersPerSecond.of(13.978), Seconds.of(1.635)),
        new ShotData.ShotEntry(Meters.of(9.692), RotationsPerSecond.of(87.000), Degrees.of(52.305),
            MetersPerSecond.of(14.350), Seconds.of(1.672)),
        new ShotData.ShotEntry(Meters.of(10.005), RotationsPerSecond.of(89.000), Degrees.of(52.305),
            MetersPerSecond.of(14.725), Seconds.of(1.708))};

    /**
     * Pre-computed shot entries for passing.
     */
    public static final ShotData.ShotEntry[] groundEntries = new ShotData.ShotEntry[] {
        new ShotData.ShotEntry(Meters.of(3.024), RotationsPerSecond.of(43.000), Degrees.of(62.305),
            MetersPerSecond.of(6.649), Seconds.of(1.122)),
        new ShotData.ShotEntry(Meters.of(3.257), RotationsPerSecond.of(44.000), Degrees.of(62.305),
            MetersPerSecond.of(6.959), Seconds.of(1.168)),
        new ShotData.ShotEntry(Meters.of(3.563), RotationsPerSecond.of(45.000), Degrees.of(62.305),
            MetersPerSecond.of(7.363), Seconds.of(1.226)),
        new ShotData.ShotEntry(Meters.of(4.165), RotationsPerSecond.of(48.000), Degrees.of(57.305),
            MetersPerSecond.of(7.643), Seconds.of(1.204)),
        new ShotData.ShotEntry(Meters.of(4.664), RotationsPerSecond.of(53.000), Degrees.of(57.305),
            MetersPerSecond.of(8.226), Seconds.of(1.282)),
        new ShotData.ShotEntry(Meters.of(5.077), RotationsPerSecond.of(54.000), Degrees.of(55.305),
            MetersPerSecond.of(8.539), Seconds.of(1.292)),
        new ShotData.ShotEntry(Meters.of(5.380), RotationsPerSecond.of(56.000), Degrees.of(53.305),
            MetersPerSecond.of(8.732), Seconds.of(1.285)),
        new ShotData.ShotEntry(Meters.of(5.759), RotationsPerSecond.of(57.000), Degrees.of(52.805),
            MetersPerSecond.of(9.113), Seconds.of(1.323)),
        new ShotData.ShotEntry(Meters.of(6.124), RotationsPerSecond.of(57.500), Degrees.of(49.305),
            MetersPerSecond.of(9.313), Seconds.of(1.284)),
        new ShotData.ShotEntry(Meters.of(6.578), RotationsPerSecond.of(61.000), Degrees.of(49.305),
            MetersPerSecond.of(9.795), Seconds.of(1.337)),
        new ShotData.ShotEntry(Meters.of(6.843), RotationsPerSecond.of(63.000), Degrees.of(49.305),
            MetersPerSecond.of(10.074), Seconds.of(1.368)),
        new ShotData.ShotEntry(Meters.of(7.145), RotationsPerSecond.of(65.000), Degrees.of(49.305),
            MetersPerSecond.of(10.399), Seconds.of(1.402)),
        new ShotData.ShotEntry(Meters.of(7.455), RotationsPerSecond.of(67.000), Degrees.of(49.305),
            MetersPerSecond.of(10.730), Seconds.of(1.437)),
        new ShotData.ShotEntry(Meters.of(7.795), RotationsPerSecond.of(69.000), Degrees.of(49.305),
            MetersPerSecond.of(11.097), Seconds.of(1.475)),
        new ShotData.ShotEntry(Meters.of(7.907), RotationsPerSecond.of(71.000), Degrees.of(52.305),
            MetersPerSecond.of(11.451), Seconds.of(1.573)),
        new ShotData.ShotEntry(Meters.of(8.217), RotationsPerSecond.of(73.000), Degrees.of(52.305),
            MetersPerSecond.of(11.803), Seconds.of(1.609)),
        new ShotData.ShotEntry(Meters.of(8.528), RotationsPerSecond.of(75.000), Degrees.of(52.305),
            MetersPerSecond.of(12.159), Seconds.of(1.645)),
        new ShotData.ShotEntry(Meters.of(8.840), RotationsPerSecond.of(77.000), Degrees.of(52.305),
            MetersPerSecond.of(12.517), Seconds.of(1.681)),
        new ShotData.ShotEntry(Meters.of(9.148), RotationsPerSecond.of(79.000), Degrees.of(52.305),
            MetersPerSecond.of(12.878), Seconds.of(1.716)),
        new ShotData.ShotEntry(Meters.of(9.453), RotationsPerSecond.of(81.000), Degrees.of(52.305),
            MetersPerSecond.of(13.242), Seconds.of(1.750)),
        new ShotData.ShotEntry(Meters.of(9.761), RotationsPerSecond.of(83.000), Degrees.of(52.305),
            MetersPerSecond.of(13.608), Seconds.of(1.785)),
        new ShotData.ShotEntry(Meters.of(10.062), RotationsPerSecond.of(85.000), Degrees.of(52.305),
            MetersPerSecond.of(13.978), Seconds.of(1.818)),
        new ShotData.ShotEntry(Meters.of(10.366), RotationsPerSecond.of(87.000), Degrees.of(52.305),
            MetersPerSecond.of(14.350), Seconds.of(1.852)),
        new ShotData.ShotEntry(Meters.of(10.666), RotationsPerSecond.of(89.000), Degrees.of(52.305),
            MetersPerSecond.of(14.725), Seconds.of(1.885)),
        new ShotData.ShotEntry(Meters.of(10.699), RotationsPerSecond.of(85.000), Degrees.of(42.305),
            MetersPerSecond.of(13.978), Seconds.of(1.567)),
        new ShotData.ShotEntry(Meters.of(11.029), RotationsPerSecond.of(87.000), Degrees.of(42.305),
            MetersPerSecond.of(14.350), Seconds.of(1.596)),
        new ShotData.ShotEntry(Meters.of(11.364), RotationsPerSecond.of(89.000), Degrees.of(42.305),
            MetersPerSecond.of(14.725), Seconds.of(1.626))};
}
