package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ClimberIO io;
    private ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
    private final BangBangController bangController = new BangBangController();

    public Climber(ClimberIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {
        Logger.processInputs("Climber/", inputs);
    }

    public Command moveToTelescope(Supplier<Distance> height) {
        return runOnce(() -> {
            Logger.recordOutput("targetHeight", height.get().in(Meters));
            io.setVoltageTelescope(bangController.calculate(inputs.positionTelescope.in(Meters),
                height.get().in(Meters)));;
        }).andThen(Commands.waitUntil(
            () -> Math.abs(inputs.positionTelescope.in(Inches) - height.get().in(Inches)) < 1));
    }


    public Command moveToPivot(Supplier<Angle> angle) {
        return runOnce(() -> {
            Logger.recordOutput("targetAngle", angle.get().in(Radians));
            io.setAnglePivot(angle.get());
        }).andThen(Commands.waitUntil(
            () -> Math.abs(inputs.positionPivot.in(Radians) - angle.get().in(Radians)) < 1));
    }
}
