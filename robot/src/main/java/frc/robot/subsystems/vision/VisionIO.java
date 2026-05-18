package frc.robot.subsystems.vision;

import java.util.Optional;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.struct.PhotonPipelineResultSerde;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.util.GenerateEmptyIO;

/** IO Class for Vision */
@GenerateEmptyIO
@NullMarked
public interface VisionIO {

    /** Inputs for an individual camera */
    public static class CameraInputs implements LoggableInputs, Cloneable {
        public PhotonPipelineResult[] results = new PhotonPipelineResult[0];
        public Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
        public Optional<Matrix<N8, N1>> distCoeffs = Optional.empty();

        private static final PhotonPipelineResultSerde serde = new PhotonPipelineResultSerde();

        @Override
        public void toLog(LogTable table) {
            table.put("length", results.length);
            for (int i = 0; i < results.length; i++) {
                LogTable subtable = table.getSubtable("[" + i + "]");
                var result = results[i];
                if (result == null) {
                    var rawBytes = new byte[0];
                    subtable.put("rawBytes", rawBytes);
                } else {
                    // https://github.com/PhotonVision/photonvision/blob/78b82e3a96d04f79e0b70746dd975c10c8ff1294/photon-core/src/main/java/org/photonvision/common/dataflow/networktables/NTDataPublisher.java#L166
                    var rawBytes = new Packet(1024);
                    serde.pack(rawBytes, result);
                    subtable.put("rawBytes", rawBytes.getWrittenDataCopy());
                }
            }
            if (cameraMatrix.isPresent()) {
                table.put("cameraMatrix", cameraMatrix.get().getData());
            } else {
                table.put("cameraMatrix", new double[0]);
            }
            if (distCoeffs.isPresent()) {
                table.put("distCoeffs", distCoeffs.get().getData());
            } else {
                table.put("distCoeffs", new double[0]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            int length = table.get("length", 0);
            results = new PhotonPipelineResult[length];
            for (int i = 0; i < length; i++) {
                LogTable subtable = table.getSubtable("[" + i + "]");
                var rawBytes = subtable.get("rawBytes", new byte[0]);
                PhotonPipelineResult result;
                if (rawBytes.length > 0) {
                    Packet p = new Packet(rawBytes);
                    result = serde.unpack(p);
                } else {
                    result = null;
                }
                results[i] = result;
            }

            double[] data = table.get("cameraMatrix", new double[0]);
            if (data.length == 0) {
                this.cameraMatrix = Optional.empty();
            } else {
                this.cameraMatrix = Optional.of(new Matrix<>(N3.instance, N3.instance, data));
            }
            data = table.get("distCoeffs", new double[0]);
            if (data.length == 0) {
                this.distCoeffs = Optional.empty();
            } else {
                this.distCoeffs = Optional.of(new Matrix<>(N8.instance, N1.instance, data));
            }
        }

    }

    public void updateInputs(CameraInputs[] inputs);

}
