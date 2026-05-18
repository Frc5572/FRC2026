package frc.gen;

public interface CameraIntrinsics {
    double fx();
    double fy();
    double cx();
    double cy();

    // k1, k2, p1, p2, k3 — return zeros if distortion is not modeled
    double[] distCoeffs();
}
