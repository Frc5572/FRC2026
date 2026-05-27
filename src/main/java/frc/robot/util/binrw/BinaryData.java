package frc.robot.util.binrw;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Utility for reading and writing {@link Binrw}-generated types to files.
 *
 * <p>
 * Pass the static {@code read} or {@code write} method reference from a generated reader/writer
 * class as the {@code binread} or {@code binwrite} argument.
 *
 * <p>
 * File paths are resolved relative to the robot's deploy directory
 * ({@link edu.wpi.first.wpilibj.Filesystem#getDeployDirectory()}).
 *
 * <pre>{@code
 * // reads deploy/data/point.bin
 * Point p = BinaryData.readFile("data/point.bin", PointReader::read);
 * BinaryData.writeFile("data/point.bin", PointWriter::write, p);
 * }</pre>
 */
public class BinaryData {

    private BinaryData() {} // utility class

    /**
     * Deserializes a value from a binary file.
     *
     * @param <T> the type to read
     */
    @FunctionalInterface
    public static interface BinRead<T> {
        T read(DataInputStream dis) throws IOException;
    }

    /**
     * Serializes a value to a binary file.
     *
     * @param <T> the type to write
     */
    @FunctionalInterface
    public static interface BinWrite<T> {
        void write(DataOutputStream dos, T value) throws IOException;
    }

    /**
     * Reads a value from a binary file using the provided reader.
     *
     * @param name path to the file, relative to the deploy directory
     * @param binread deserialization function, typically a generated {@code Reader::read} reference
     * @return the deserialized value
     * @throws IOException if the file cannot be read or deserialization fails
     */
    public static <T> T readFile(String name, BinRead<T> binread) throws IOException {
        FileInputStream fis = new FileInputStream(new File(Filesystem.getDeployDirectory(), name));
        DataInputStream dis = new DataInputStream(fis);
        return binread.read(dis);
    }

    /**
     * Writes a value to a binary file using the provided writer.
     *
     * @param name path to the file, relative to the deploy directory
     * @param binwrite serialization function, typically a generated {@code Writer::write} reference
     * @param value the value to serialize
     * @throws IOException if the file cannot be written or serialization fails
     */
    public static <T> void writeFile(String name, BinWrite<T> binwrite, T value)
        throws IOException {
        FileOutputStream fos =
            new FileOutputStream(new File(Filesystem.getDeployDirectory(), name));
        DataOutputStream dos = new DataOutputStream(fos);
        binwrite.write(dos, value);
    }

}
