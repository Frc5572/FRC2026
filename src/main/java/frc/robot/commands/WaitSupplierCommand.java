package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitSupplierCommand extends Command {
    private final DoubleSupplier timeSupplier;
    private final Timer timer = new Timer();
    private double targetTime;

    public WaitSupplierCommand(DoubleSupplier timeSupplier) {
        this.timeSupplier = timeSupplier;
    }

    @Override
    public void initialize() {
        targetTime = timeSupplier.getAsDouble(); // snapshot here
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= targetTime;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
