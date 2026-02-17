package frc.robot.util.tunable;

import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

public class FlywheelConstants implements LoggableInputs, Cloneable, Tunable {

    public double holdCurrent;
    public double maxDutyCycle;
    public boolean isReversed;
    public double velocityTolerance;
    public double atSpeedDebounce;
    private boolean isDirty;

    @TypeStateBuilder
    public FlywheelConstants(@RequiredField double holdCurrent, @RequiredField double maxDutyCycle,
        @RequiredField boolean isReversed, @RequiredField double velocityTolerance,
        @RequiredField double atSpeedDebounce) {
        this.holdCurrent = holdCurrent;
        this.maxDutyCycle = maxDutyCycle;
        this.isReversed = isReversed;
        this.velocityTolerance = velocityTolerance;
        this.atSpeedDebounce = atSpeedDebounce;

        this.isDirty = false;

        Tunable.setupTunable("/Flywheel", this, FlywheelConstants.class, () -> {
            this.isDirty = true;
        });
    }

    public void ifDirty(Consumer<FlywheelConstants> consumer) {
        Logger.processInputs("Flywheel", this);
        if (this.isDirty) {
            consumer.accept(this);
        }
        this.isDirty = false;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("maxTorque", this.holdCurrent);
        table.put("maxDutyCycle", this.maxDutyCycle);
        table.put("isReversed", this.isReversed);
    }

    @Override
    public void fromLog(LogTable table) {
        this.holdCurrent = table.get("maxTorque", this.holdCurrent);
        this.maxDutyCycle = table.get("maxDutyCycle", this.maxDutyCycle);
        this.isReversed = table.get("isReversed", this.isReversed);
    }



}
