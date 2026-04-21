# Procedure for regenerating shot data

1. In AdvantageScope: connect to the robot
2. Create a Line Graph view
3. Pull `/AdvantageKit/RealOutputs/ShotParameters/distance` into the Left Axis table
4. Convert the Left Axis from Meters to Feet
  a. Click the three dots > `Manual Units` > `Edit Conversion...`
  b. Set Unit Type to `length`
  c. Set `from` to `meters`
  d. Set `to` to `feet`
5. In the left panel, scroll all the way to the top. Click the three bars next to the search bar.
6. Scroll down to `ShotDataHelper`
7. Modify `flywheelSpeed` and `hoodAngle`
8. On controller 2, hold right trigger to spin up the shooter, and left trigger to run the spindexer.
9. Once you have a triple that scores, go to `ShotData.java` (in this folder).
10. Add an entry to `entries` using the distance (in feet), the flywheel speed (as entered), and the hood angle (as entered). The final parameter can be `0`.
11. Once entries are finalized, regenerate the LUTs
  a. Open a terminal in the main directory.
  b. Run `./gradlew generateLUTs`.