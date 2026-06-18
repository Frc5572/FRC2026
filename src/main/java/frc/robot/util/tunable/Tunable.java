package frc.robot.util.tunable;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Generate a <Class>Tunable class that automatically generates NT-based tuner. */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.SOURCE)
public @interface Tunable {
}
