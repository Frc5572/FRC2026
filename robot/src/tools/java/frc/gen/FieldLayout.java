package frc.gen;

public interface FieldLayout {
    // Returns 4 corners [x,y,z] in field coordinates (meters), in ArUco order:
    // top-left, top-right, bottom-right, bottom-left (when viewed from front).
    // Returns null if the tag ID is not known.
    double[][] getTagCorners(int tagId);
}
