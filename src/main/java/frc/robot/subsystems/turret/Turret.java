package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();

    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Logger.recordOutput("Turret/Gear1Angle", inputs.gear1AbsoluteAngle);
        Logger.recordOutput("Turret/Gear2Angle", inputs.gear2AbsoluteAngle);

        // TODO calculate turret angle from gear angles
    }

}
