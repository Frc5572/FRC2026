package frc.robot.util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Generate Empty IO implementation using RobotUtils */
@Retention(RetentionPolicy.CLASS)
@Target(ElementType.TYPE)
public @interface GenerateEmptyIO {

    /** Parameter types for constructor */
    public Class<?>[] value() default {};

}
