package frc.gen;

// Camera pose in field coordinates, derived from solvePnP.
// translation: camera position (x, y, z) in meters in field frame.
// rvec: camera orientation as a Rodrigues rotation vector in field frame.
// detectedTagIds: IDs of the tags used to compute this pose.
public record PoseResult(int[] detectedTagIds, double[] translation, double[] rvec) {}
