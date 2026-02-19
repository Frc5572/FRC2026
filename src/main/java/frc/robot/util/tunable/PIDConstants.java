package frc.robot.util.tunable;

import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.util.typestate.InitField;
import frc.robot.util.typestate.OptionalField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Tunable constants for PID controllers */
public class PIDConstants implements LoggableInputs, Cloneable, Tunable {

    public double kP;
    public double kD;
    public double kI;
    public double kV;
    public double kS;
    public double kG;
    public double kA;
    public boolean isArm;
    private boolean isDirty;
    private String name;

    /** Create new PID Constants */
    @TypeStateBuilder("PIDConstantsBuilder")
    public PIDConstants(@InitField String name, @InitField GravityTypeValue gravityType,
        @RequiredField double kP, @OptionalField("0.0") double kI, @OptionalField("0.0") double kD,
        @OptionalField("0.0") double kV, @OptionalField("0.0") double kS,
        @OptionalField("0.0") double kG, @OptionalField("0.0") double kA) {
        this.name = name;
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kV = kV;
        this.kS = kS;
        this.kG = kG;
        this.kA = kA;
        this.isArm = gravityType.equals(GravityTypeValue.Arm_Cosine);
        this.isDirty = true;

        Tunable.setupTunable("/" + name, this, PIDConstants.class, () -> {
            this.isDirty = true;
        });
    }

    /** Write PID constants to TalonFX */
    public void apply(Slot0Configs config) {
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kV;
        config.kG = kG;
        config.kS = kS;
        config.kA = kA;
        config.GravityType = isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
    }

    /** Run function with constants if they've changed. */
    public void ifDirty(Consumer<PIDConstants> consumer) {
        Logger.processInputs(this.name, this);
        if (this.isDirty) {
            consumer.accept(this);
        }
        this.isDirty = false;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("kP", this.kP);
        table.put("kI", this.kI);
        table.put("kD", this.kD);
        table.put("kV", this.kV);
        table.put("kG", this.kG);
        table.put("kS", this.kS);
        table.put("kA", this.kA);
        table.put("isArm", this.isArm);
    }

    @Override
    public void fromLog(LogTable table) {
        this.kP = table.get("kP", this.kP);
        this.kI = table.get("kI", this.kI);
        this.kD = table.get("kD", this.kD);
        this.kV = table.get("kV", this.kV);
        this.kG = table.get("kG", this.kG);
        this.kS = table.get("kS", this.kS);
        this.kS = table.get("kA", this.kA);
        this.isArm = table.get("isArm", this.isArm);
    }

    @Override
    public PIDConstants clone() {
        PIDConstants copy = new PIDConstants(name,
            isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static, kP, kD, kI, kV,
            kS, kG, kA);
        return copy;
    }

}
