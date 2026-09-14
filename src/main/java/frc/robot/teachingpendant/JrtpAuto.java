package frc.robot.teachingpendant;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

/** Portable JSON representation of a ROSBots teaching-pendant autonomous routine. */
@JsonIgnoreProperties(ignoreUnknown = true)
public final class JrtpAuto {
    public String format = "jrtp";
    public int version = 1;
    public String name = "Untitled Auto";
    public double maximumTime = 15.0;
    public Pose startingPose = new Pose();
    public List<Step> steps = new ArrayList<>();

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static final class Pose {
        public double x;
        public double y;
        public double rotationDegrees;

        public Pose() {}

        public Pose(double x, double y, double rotationDegrees) {
            this.x = x;
            this.y = y;
            this.rotationDegrees = rotationDegrees;
        }
    }

    /** A step is deliberately extensible so newer pendant versions remain loadable by old robots. */
    @JsonIgnoreProperties(ignoreUnknown = true)
    public static final class Step {
        public String type;
        public String name;
        public Double x;
        public Double y;
        public Double rotationDegrees;
        public Double maxSpeed;
        public String command;
        public Map<String, Object> parameters = new LinkedHashMap<>();
        public Map<String, Input> inputs = new LinkedHashMap<>();
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static final class Input {
        public String type;
        public Object defaultValue;
        public List<Object> options = new ArrayList<>();
        public Double minimum;
        public Double maximum;

        /** Accepts the specification's JSON key named \"default\". */
        @com.fasterxml.jackson.annotation.JsonProperty("default")
        public void setDefaultValue(Object value) {
            defaultValue = value;
        }

        @com.fasterxml.jackson.annotation.JsonProperty("default")
        public Object getDefaultValue() {
            return defaultValue;
        }
    }
}
