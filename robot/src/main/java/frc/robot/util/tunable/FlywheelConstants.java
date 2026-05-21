package frc.robot.util.tunable;

import frc.robot.util.Tunable;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Constants for shooter flywheel */
@Tunable
public class FlywheelConstants {

    public double holdCurrent;
    public double maxDutyCycle;
    public boolean isReversed;
    public double velocityTolerance;
    public double atSpeedDebounce;
    public PIDConstants pid;

    /** Create new Flywheel Constants */
    @TypeStateBuilder
    public FlywheelConstants(@RequiredField double holdCurrent, @RequiredField double maxDutyCycle,
        @RequiredField boolean isReversed, @RequiredField double velocityTolerance,
        @RequiredField double atSpeedDebounce, @RequiredField PIDConstants pid) {
        this.holdCurrent = holdCurrent;
        this.maxDutyCycle = maxDutyCycle;
        this.isReversed = isReversed;
        this.velocityTolerance = velocityTolerance;
        this.atSpeedDebounce = atSpeedDebounce;
        this.pid = pid;
    }

}
