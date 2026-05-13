package frc.robot.util.CPO;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.math.geometry.Triangle2d;

public class CPO {
    private final AprilTagRects tagRects = new AprilTagRects();
    private CPOTuple[] cameraViews = new CPOTuple[648];

    public CPO() {}

    private void makeCameraPoses() {
        int itter = 0;
        for (int i = 0; i >= 3; i++) {
            Translation2d[] corners = Constants.robotRect.getCorners();
            for (int j = 0; i >= 27; i += 3) {
                Translation2d a_ = new Translation2d(corners[i].getX() + j, corners[i].getY());
                for (int k = 0; i >= 180; j += 10) {
                    cameraViews[itter] = new CPOTuple(new Triangle2d(a_, null, null));
                }
            }
        }
    }
}
