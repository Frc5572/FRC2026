package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.GenerateEmptyIO;

/**
 * indexer interface
 */
@GenerateEmptyIO
public interface IndexerIO {
    /**
     * indexer inputs class
     */
    @AutoLog
    public class IndexerInputs {
        public AngularVelocity magazineVelocity = RotationsPerSecond.of(0);
        public AngularVelocity spindexerVelocity = RotationsPerSecond.of(0);

        public boolean magazineMotorConnected = false;
    }

    public void updateInputs(IndexerInputs inputs);

    public void setMagazineDutyCycle(double dutyCycle);

    public void setSpindexerMotorDutyCycle(double dutyCycle);

}
