package frc.robot.util.binrw;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Generates symmetric binary reader and writer classes for a class or record type.
 *
 * <p>
 * Applying this annotation causes the annotation processor to emit two classes in the same package:
 *
 * <ul>
 * <li>{@code <TypeName>Reader} - a final class with a static {@code read(DataInputStream)} method
 * that deserializes an instance from a stream.
 * <li>{@code <TypeName>Writer} - a final class with a static {@code write(DataOutputStream, T)}
 * method that serializes an instance to a stream.
 * </ul>
 *
 * <p>
 * Serialization is derived from the type's public fields (for classes) or record components (for
 * records, accessed via their accessor methods). Supported field types and their wire encoding:
 *
 * <ul>
 * <li>Primitives ({@code int}, {@code double}, etc.) - the matching {@code DataInputStream}
 * read/write method.
 * <li>{@link String} - {@code readUTF} / {@code writeUTF}.
 * <li>Arrays and {@link java.util.List} - 32-bit element count followed by each element. Lists are
 * deserialized into {@code ArrayList}.
 * <li>{@link java.util.Map} - 32-bit entry count followed by alternating key/value pairs. Maps are
 * deserialized into {@code LinkedHashMap}.
 * <li>Types annotated with {@code @Binrw} - delegates to their generated reader/writer.
 * </ul>
 *
 * <p>
 * Fields or record components annotated with {@link BrwIgnore} are excluded from serialization
 * entirely.
 *
 * <p>
 * Example:
 *
 * <pre>{@code
 * @Binrw
 * public record Point(double x, double y) {
 * }
 * // generates PointReader.read(DataInputStream) and PointWriter.write(DataOutputStream, Point)
 * }</pre>
 *
 * @see BrwIgnore
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.SOURCE)
public @interface Binrw {
    /**
     * An implementation class that provides hand-written {@code read} and {@code write} static
     * methods. When set, the generated reader/writer simply delegate to {@code Impl.read(stream)}
     * and {@code Impl.write(stream, obj)} instead of deriving serialization from the type's fields.
     * Defaults to {@code void.class}, which means the processor generates implementations
     * automatically.
     */
    public Class<?> value() default void.class;
}
