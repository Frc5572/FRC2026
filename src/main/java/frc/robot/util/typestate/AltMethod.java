package frc.robot.util.typestate;

/** Alternative builder step. */
public @interface AltMethod {

    /** The alternative parameter type. */
    public Class<?> type();

    /** If specified, this parameter name is used instead of the constructor parameter name. */
    public String parameter_name() default "";

    /** Stringified java expression converting the alternative type into the actual type. */
    public String value();

}
