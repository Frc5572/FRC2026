package frc.robot.util.typestate;

/** Marks this parameter as optional. */
public @interface OptionalField {

    /** Stringified java expression used when this parameter is not specified. */
    public String value();

    /** Alternative build methods. */
    public AltMethod[] alt() default {};

}
