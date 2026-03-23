package frc.robot.shotdata;

public class ProjectileSim {

    public record TrajectoryResult(double zAtTarget, double tof, boolean reachedTarget,
        double maxHeight, double apexX) {
    }

    public TrajectoryResult simulate(double rpm, double targetDistanceM, double launchAngleDeg) {
        // TODO
        // https://github.com/eeveemara/frc-fire-control/blob/main/src/main/java/frc/firecontrol/ProjectileSimulator.java
        return null;
    }

}
