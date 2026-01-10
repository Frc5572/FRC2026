package frc.robot.util.typestate;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Generate TypeState Builder using RobotUtils */
@Retention(RetentionPolicy.CLASS)
@Target(ElementType.CONSTRUCTOR)
public @interface TypeStateBuilder {

    /**
     * If specified, the builder's class name is this. Otherwise, "Builder" is appended to the
     * enclosing class's name.
     */
    public String value() default "";

}
