package frc.gen;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.bytedeco.javacpp.indexer.DoubleIndexer;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.javacpp.indexer.IntIndexer;
import org.bytedeco.javacv.Java2DFrameConverter;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_objdetect.ArucoDetector;
import org.bytedeco.opencv.opencv_objdetect.Dictionary;
import static org.bytedeco.opencv.global.opencv_calib3d.Rodrigues;
import static org.bytedeco.opencv.global.opencv_calib3d.solvePnP;
import static org.bytedeco.opencv.global.opencv_core.CV_64F;
import static org.bytedeco.opencv.global.opencv_core.gemm;
import static org.bytedeco.opencv.global.opencv_core.transpose;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_BGR2GRAY;
import static org.bytedeco.opencv.global.opencv_imgproc.cvtColor;
import static org.bytedeco.opencv.global.opencv_objdetect.DICT_APRILTAG_36h11;
import static org.bytedeco.opencv.global.opencv_objdetect.getPredefinedDictionary;

public class PoseEstimator {

    private final FieldLayout fieldLayout;
    private CameraIntrinsics intrinsics;
    private Mat cameraMatrix;
    private Mat distCoeffs;

    private final ArucoDetector detector;
    private final Java2DFrameConverter frameConverter = new Java2DFrameConverter();
    private final OpenCVFrameConverter.ToMat matConverter = new OpenCVFrameConverter.ToMat();

    public PoseEstimator(FieldLayout fieldLayout) {
        this(null, fieldLayout);
    }

    public PoseEstimator(CameraIntrinsics intrinsics, FieldLayout fieldLayout) {
        this.fieldLayout = fieldLayout;
        if (intrinsics != null) setIntrinsics(intrinsics);

        Dictionary dict = getPredefinedDictionary(DICT_APRILTAG_36h11);
        this.detector = new ArucoDetector(dict);
    }

    public void setIntrinsics(CameraIntrinsics intrinsics) {
        this.intrinsics = intrinsics;
        this.cameraMatrix = buildCameraMatrix(intrinsics);
        this.distCoeffs = buildDistCoeffs(intrinsics);
    }

    public Optional<PoseResult> estimate(BufferedImage image) {
        if (intrinsics == null) {
            setIntrinsics(new AssumedCameraIntrinsics(image.getWidth(), image.getHeight()));
        }

        Mat colorMat = matConverter.convert(frameConverter.convert(image));
        Mat grayMat = new Mat();
        cvtColor(colorMat, grayMat, COLOR_BGR2GRAY);

        MatVector corners = new MatVector();
        Mat ids = new Mat();
        detector.detectMarkers(grayMat, corners, ids, new MatVector());

        if (ids.empty()) return Optional.empty();

        List<double[]> objPts = new ArrayList<>();
        List<double[]> imgPts = new ArrayList<>();
        List<Integer> detectedIds = new ArrayList<>();

        IntIndexer idsIdx = ids.createIndexer();
        for (int i = 0; i < ids.rows(); i++) {
            int tagId = idsIdx.get(i, 0);
            double[][] tagCorners = fieldLayout.getTagCorners(tagId);
            if (tagCorners == null) continue;

            detectedIds.add(tagId);
            for (double[] corner : tagCorners) objPts.add(corner);

            Mat cornerMat = corners.get(i);
            FloatIndexer fi = cornerMat.createIndexer();
            for (int c = 0; c < 4; c++) {
                imgPts.add(new double[]{fi.get(0L, c, 0), fi.get(0L, c, 1)});
            }
            fi.release();
        }
        idsIdx.release();

        if (detectedIds.isEmpty()) return Optional.empty();

        Mat objectPoints = toMat(objPts, 3);
        Mat imagePoints = toMat(imgPts, 2);
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        // Invert solvePnP output to get camera pose in field coordinates.
        // solvePnP gives R, t such that p_cam = R*p_world + t.
        // Camera position in world: -R^T * t.
        // Camera orientation in world: R^T (stored as rvec via Rodrigues).
        Mat R = new Mat();
        Rodrigues(rvec, R);
        Mat R_T = new Mat();
        transpose(R, R_T);
        Mat camPos = new Mat();
        gemm(R_T, tvec, -1.0, new Mat(), 0.0, camPos);
        Mat rvecWorld = new Mat();
        Rodrigues(R_T, rvecWorld);

        return Optional.of(new PoseResult(
            detectedIds.stream().mapToInt(x -> x).toArray(),
            extractDoubles(camPos),
            extractDoubles(rvecWorld)
        ));
    }

    private static Mat buildCameraMatrix(CameraIntrinsics i) {
        Mat m = new Mat(3, 3, CV_64F);
        DoubleIndexer idx = m.createIndexer();
        idx.put(0, 0, i.fx());  idx.put(0, 1, 0.0);     idx.put(0, 2, i.cx());
        idx.put(1, 0, 0.0);     idx.put(1, 1, i.fy());   idx.put(1, 2, i.cy());
        idx.put(2, 0, 0.0);     idx.put(2, 1, 0.0);      idx.put(2, 2, 1.0);
        idx.release();
        return m;
    }

    private static Mat buildDistCoeffs(CameraIntrinsics i) {
        double[] d = i.distCoeffs();
        Mat m = new Mat(1, d.length, CV_64F);
        DoubleIndexer idx = m.createIndexer();
        for (int j = 0; j < d.length; j++) idx.put(0, j, d[j]);
        idx.release();
        return m;
    }

    private static Mat toMat(List<double[]> pts, int cols) {
        Mat m = new Mat(pts.size(), cols, CV_64F);
        DoubleIndexer idx = m.createIndexer();
        for (int i = 0; i < pts.size(); i++)
            for (int j = 0; j < cols; j++)
                idx.put(i, j, pts.get(i)[j]);
        idx.release();
        return m;
    }

    private static double[] extractDoubles(Mat m) {
        DoubleIndexer idx = m.createIndexer();
        double[] out = new double[3];
        for (int i = 0; i < 3; i++) out[i] = idx.get(i);
        idx.release();
        return out;
    }
}
