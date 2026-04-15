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
    new ShotData.ShotEntry(Meters.of(3.088), RotationsPerSecond.of(51.000), Degrees.of(57.305),
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
    new ShotData.ShotEntry(Meters.of(4.979), RotationsPerSecond.of(67.000), Degrees.of(52.305),
      MetersPerSecond.of(9.282), Seconds.of(1.072)),
    new ShotData.ShotEntry(Meters.of(5.185), RotationsPerSecond.of(69.000), Degrees.of(52.305),
      MetersPerSecond.of(9.487), Seconds.of(1.102)),
    new ShotData.ShotEntry(Meters.of(5.387), RotationsPerSecond.of(71.000), Degrees.of(52.305),
      MetersPerSecond.of(9.688), Seconds.of(1.131)),
    new ShotData.ShotEntry(Meters.of(5.585), RotationsPerSecond.of(73.000), Degrees.of(52.305),
      MetersPerSecond.of(9.883), Seconds.of(1.159)),
    new ShotData.ShotEntry(Meters.of(5.772), RotationsPerSecond.of(75.000), Degrees.of(52.305),
      MetersPerSecond.of(10.073), Seconds.of(1.185)),
    new ShotData.ShotEntry(Meters.of(5.955), RotationsPerSecond.of(77.000), Degrees.of(52.305),
      MetersPerSecond.of(10.258), Seconds.of(1.210)),
    new ShotData.ShotEntry(Meters.of(6.128), RotationsPerSecond.of(79.000), Degrees.of(52.305),
      MetersPerSecond.of(10.438), Seconds.of(1.233)),
    new ShotData.ShotEntry(Meters.of(6.299), RotationsPerSecond.of(81.000), Degrees.of(52.305),
      MetersPerSecond.of(10.613), Seconds.of(1.256)),
    new ShotData.ShotEntry(Meters.of(6.465), RotationsPerSecond.of(83.000), Degrees.of(52.305),
      MetersPerSecond.of(10.783), Seconds.of(1.278)),
    new ShotData.ShotEntry(Meters.of(6.624), RotationsPerSecond.of(85.000), Degrees.of(52.305),
      MetersPerSecond.of(10.947), Seconds.of(1.299)),
    new ShotData.ShotEntry(Meters.of(6.773), RotationsPerSecond.of(87.000), Degrees.of(52.305),
      MetersPerSecond.of(11.107), Seconds.of(1.318)),
    new ShotData.ShotEntry(Meters.of(6.921), RotationsPerSecond.of(89.000), Degrees.of(52.305),
      MetersPerSecond.of(11.261), Seconds.of(1.337))};

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
    new ShotData.ShotEntry(Meters.of(4.045), RotationsPerSecond.of(51.000), Degrees.of(57.305),
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
    new ShotData.ShotEntry(Meters.of(5.942), RotationsPerSecond.of(67.000), Degrees.of(52.305),
      MetersPerSecond.of(9.282), Seconds.of(1.334)),
    new ShotData.ShotEntry(Meters.of(6.132), RotationsPerSecond.of(69.000), Degrees.of(52.305),
      MetersPerSecond.of(9.487), Seconds.of(1.358)),
    new ShotData.ShotEntry(Meters.of(6.315), RotationsPerSecond.of(71.000), Degrees.of(52.305),
      MetersPerSecond.of(9.688), Seconds.of(1.381)),
    new ShotData.ShotEntry(Meters.of(6.493), RotationsPerSecond.of(73.000), Degrees.of(52.305),
      MetersPerSecond.of(9.883), Seconds.of(1.403)),
    new ShotData.ShotEntry(Meters.of(6.665), RotationsPerSecond.of(75.000), Degrees.of(52.305),
      MetersPerSecond.of(10.073), Seconds.of(1.424)),
    new ShotData.ShotEntry(Meters.of(6.835), RotationsPerSecond.of(77.000), Degrees.of(52.305),
      MetersPerSecond.of(10.258), Seconds.of(1.445)),
    new ShotData.ShotEntry(Meters.of(6.999), RotationsPerSecond.of(79.000), Degrees.of(52.305),
      MetersPerSecond.of(10.438), Seconds.of(1.465)),
    new ShotData.ShotEntry(Meters.of(7.157), RotationsPerSecond.of(81.000), Degrees.of(52.305),
      MetersPerSecond.of(10.613), Seconds.of(1.484)),
    new ShotData.ShotEntry(Meters.of(7.309), RotationsPerSecond.of(83.000), Degrees.of(52.305),
      MetersPerSecond.of(10.783), Seconds.of(1.502)),
    new ShotData.ShotEntry(Meters.of(7.458), RotationsPerSecond.of(85.000), Degrees.of(52.305),
      MetersPerSecond.of(10.947), Seconds.of(1.520)),
    new ShotData.ShotEntry(Meters.of(7.601), RotationsPerSecond.of(87.000), Degrees.of(52.305),
      MetersPerSecond.of(11.107), Seconds.of(1.537)),
    new ShotData.ShotEntry(Meters.of(7.738), RotationsPerSecond.of(89.000), Degrees.of(52.305),
      MetersPerSecond.of(11.261), Seconds.of(1.553)),
    new ShotData.ShotEntry(Meters.of(7.858), RotationsPerSecond.of(85.000), Degrees.of(42.305),
      MetersPerSecond.of(10.947), Seconds.of(1.305)),
    new ShotData.ShotEntry(Meters.of(8.014), RotationsPerSecond.of(87.000), Degrees.of(42.305),
      MetersPerSecond.of(11.107), Seconds.of(1.320)),
    new ShotData.ShotEntry(Meters.of(8.162), RotationsPerSecond.of(89.000), Degrees.of(42.305),
      MetersPerSecond.of(11.261), Seconds.of(1.334))};
}
