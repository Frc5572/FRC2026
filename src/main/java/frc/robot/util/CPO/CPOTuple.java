package frc.robot.util.CPO;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.math.geometry.Triangle2d;

public class CPOTuple {
    Triangle2d camera;
    int singleTagViews = 0;
    int doubleTagViews = 0;

    public CPOTuple(Triangle2d triangle) {
        this.camera = triangle;
    }

    public void incrementDecrement(int single, int double_) {
        singleTagViews += single;
        doubleTagViews += double_;
    }

    public double[] getter() {
        return new double[] {singleTagViews, doubleTagViews};
    }

    public void updateTriangle(Pose2d pose) {
        camera.transformBy(pose);
    }
}
