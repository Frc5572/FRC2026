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

    /**
     * If true, required fields must be specified in the same order as the constructor. If false,
     * required fields may be specified in any order. Defaults to false.
     */
    public boolean linear() default false;

}
