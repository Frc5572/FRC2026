package frc.robot.util.typestate;

/** Field required to be specified before finishing. */
public @interface RequiredField {

    /** Alternative build methods. */
    public AltMethod[] alt() default {};
}
