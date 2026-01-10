package frc.robot.util.typestate;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Marks this parameter as required to create the builder. */
@Retention(RetentionPolicy.CLASS)
@Target(ElementType.PARAMETER)
public @interface InitField {
}
