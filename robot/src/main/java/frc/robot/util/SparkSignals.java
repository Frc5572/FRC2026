package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

/**
 * Utility methods for safely interacting with REV Spark motor controllers in the presence of
 * transient or persistent communication errors.
 *
 * <p>
 * REV API calls report errors via {@link REVLibError} rather than exceptions. These helpers
 * centralize common error-handling patterns and ensure that invalid or stale values are not
 * inadvertently consumed by higher-level logic.
 *
 * <p>
 * All utility methods update a shared {@link #sparkStickyFault} flag whenever a non-{@code kOk}
 * error is detected. This flag may be monitored elsewhere in the robot code to trigger diagnostics,
 * alerts, or safe fallback behavior.
 *
 * <p>
 * This class is purely static and is not intended to be instantiated.
 */
@NullMarked
public class SparkSignals {

    private SparkSignals() {}

    /**
     * Sticky indicator that is set whenever a Spark-related error is detected.
     *
     * <p>
     * Once set, this flag remains {@code true} until explicitly cleared by user code. It can be
     * used to surface non-fatal communication issues that may otherwise go unnoticed during
     * operation.
     */
    public static boolean sparkStickyFault = false;

    /**
     * Retrieves and processes a value from a Spark controller only if the underlying call completed
     * successfully.
     *
     * <p>
     * If the Spark reports an error, the value is discarded and the consumer is not invoked.
     *
     * @param spark the Spark device associated with the value
     * @param supplier supplier that retrieves the value from the Spark
     * @param consumer consumer that processes the value if it is valid
     */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /**
     * Retrieves multiple values from a Spark controller and processes them only if all values are
     * valid.
     *
     * <p>
     * If any supplier produces an error, processing is aborted and the consumer is not invoked.
     *
     * @param spark the Spark device associated with the values
     * @param suppliers array of suppliers retrieving Spark values
     * @param consumer consumer that processes the values if all are valid
     */
    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers,
        Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /**
     * Retrieves a value from a Spark controller, returning a default if the value is invalid.
     *
     * <p>
     * If an error occurs, the default value is returned and the sticky fault flag is set.
     *
     * @param spark the Spark device associated with the value
     * @param supplier supplier that retrieves the value
     * @param defaultValue value to return if an error is detected
     * @return the retrieved value if valid, otherwise {@code defaultValue}
     */
    public static double ifOkOrDefault(SparkBase spark, DoubleSupplier supplier,
        double defaultValue) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            return value;
        } else {
            sparkStickyFault = true;
            return defaultValue;
        }
    }

    /**
     * Retrieves multiple values from a Spark controller and applies a transformation only if all
     * values are valid.
     *
     * <p>
     * If any value is invalid, the transformation is skipped and a default value is returned.
     *
     * @param spark the Spark device associated with the values
     * @param suppliers array of suppliers retrieving Spark values
     * @param transformer function that computes a result from the retrieved values
     * @param defaultValue value to return if any supplier reports an error
     * @return transformed value if all inputs are valid, otherwise {@code defaultValue}
     */
    public static double ifOkOrDefault(SparkBase spark, DoubleSupplier[] suppliers,
        Function<double[], Double> transformer, double defaultValue) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return defaultValue;
            }
        }
        return transformer.apply(values);
    }

    /**
     * Repeatedly executes a Spark command until it succeeds or the maximum number of attempts is
     * reached.
     *
     * <p>
     * This is commonly used for configuration calls that may fail during initialization due to
     * transient CAN issues.
     *
     * @param maxAttempts maximum number of attempts before giving up
     * @param command supplier that executes the Spark command and returns a {@link REVLibError}
     */
    public static void tryUntilOk(int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error == REVLibError.kOk) {
                break;
            } else {
                sparkStickyFault = true;
            }
        }
    }

}
