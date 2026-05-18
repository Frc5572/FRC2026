package frc.gen;

// Estimates intrinsics from image dimensions alone.
// Assumes fx = fy = image width (≈53° horizontal FOV) and no lens distortion.
public class AssumedCameraIntrinsics implements CameraIntrinsics {

    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;
    private static final double[] ZERO_DIST = new double[5];

    public AssumedCameraIntrinsics(int imageWidth, int imageHeight) {
        this.fx = imageWidth;
        this.fy = imageWidth;
        this.cx = imageWidth / 2.0;
        this.cy = imageHeight / 2.0;
    }

    @Override public double fx() { return fx; }
    @Override public double fy() { return fy; }
    @Override public double cx() { return cx; }
    @Override public double cy() { return cy; }
    @Override public double[] distCoeffs() { return ZERO_DIST; }
}
