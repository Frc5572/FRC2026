package frc.robot.controls;

import org.jspecify.annotations.NullMarked;

/**
 * Which set of driver bindings is in force.
 *
 * <p>
 * A scheme changes <em>where</em> an input comes from, not how it is shaped &mdash; deadbands,
 * response curves and speed caps are shared, so switching schemes does not silently re-tune the
 * robot underneath the driver.
 *
 * <p>
 * Both schemes' bindings are registered at startup, each gated on the active scheme, so a driver
 * can switch between them from the tuner without a redeploy.
 */
@NullMarked
public enum ControlScheme {

    /**
     * Turning on the right stick, shooting on the right trigger, intaking on the left trigger. This
     * is the long-standing layout.
     */
    STICK_TURN("Stick turning", "Turn: right stick - Shoot: RT - Intake: LT"),

    /**
     * Turning on the triggers, with the face buttons taking over the game-piece actions. The turn
     * rate is the left trigger minus the right, so left turns counterclockwise and pulling both
     * cancels out.
     */
    TRIGGER_TURN("Trigger turning", "Turn: LT/RT - Shoot: A - Intake: B");

    private final String label;
    private final String summary;

    ControlScheme(String label, String summary) {
        this.label = label;
        this.summary = summary;
    }

    /** Short human-readable name, shown in the tuner. */
    public String label() {
        return label;
    }

    /** One-line description of the bindings, shown in the tuner. */
    public String summary() {
        return summary;
    }

    /** The scheme used when a profile does not name one. */
    public static ControlScheme defaultScheme() {
        return STICK_TURN;
    }

    /**
     * Parse a scheme name, falling back to the default rather than throwing.
     *
     * @param name a scheme name, matched case-insensitively
     * @return the matching scheme, or {@link #defaultScheme()} if the name is not recognised
     */
    public static ControlScheme fromName(String name) {
        for (ControlScheme scheme : values()) {
            if (scheme.name().equalsIgnoreCase(name)) {
                return scheme;
            }
        }
        return defaultScheme();
    }

}
