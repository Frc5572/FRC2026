package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Field geometry and reference points for path planning, vision, and alignment.
 *
 * Coordinate system: - All constants are expressed in the WPILib field coordinate system. - Values
 * are defined from the perspective of the BLUE alliance wall. (That is, you should use WPILib's
 * standard field coordinates where +X increases away from the blue wall and +Y increases to the
 * left when looking from the blue alliance station toward the field.)
 *
 * Source of truth: - Field size and AprilTag poses are loaded from an {@link AprilTagFieldLayout}
 * JSON in the deploy directory (or from source when HAL is disabled).
 */
public class FieldConstants {
    /**
     * Which physical field variant the geometry files correspond to.
     *
     * This value affects which deploy subfolder is used when loading AprilTag layouts (see
     * {@link FieldType#getJsonFolder()}).
     */
    public static final FieldType fieldType = FieldType.WELDED;

    // AprilTag related constants

    /**
     * Number of AprilTags in the currently selected official layout.
     *
     * This is computed from {@link AprilTagLayoutType#OFFICIAL}.
     */
    public static final int aprilTagCount =
        AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();

    /**
     * Physical width of an AprilTag (edge length) in meters.
     *
     * Units: meters.
     */
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);

    /**
     * Default AprilTag layout to use for most robot code.
     *
     * Note: other layouts (such as {@link AprilTagLayoutType#NONE}) can exist for
     * simulation/testing.
     */
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    // Field dimensions

    /**
     * Field length in meters as defined by the official AprilTag layout.
     *
     * Units: meters.
     */
    public static final double fieldLength =
        AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();

    /**
     * Field width in meters as defined by the official AprilTag layout.
     *
     * Units: meters.
     */
    public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    /**
     * Convenient X positions for important vertical (field-lengthwise) lines.
     *
     * Vertical lines are represented as X offsets in field coordinates. Many are derived from
     * AprilTag X positions for robustness to year/layout.
     */
    public static class LinesVertical {
        /** Field centerline X (half the field length). */
        public static final double center = fieldLength / 2.0;

        /**
         * Starting line X on the alliance side, derived from the hub/tag system.
         *
         * Note: This uses the X of tag 26 in the official layout.
         */
        public static final double starting =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();

        /** Alias for the alliance zone boundary line X. */
        public static final double allianceZone = starting;

        /**
         * Hub center X on the alliance side.
         *
         * Computed as tag 26 X plus half the hub width.
         */
        public static final double hubCenter =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;

        /** Near edge of the neutral zone (toward blue). */
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);

        /** Far edge of the neutral zone (toward red). */
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);

        /**
         * Hub center X on the opposing side.
         *
         * Computed as tag 4 X plus half the hub width.
         */
        public static final double oppHubCenter =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;

        /**
         * Opposing alliance zone boundary line X (near the far wall).
         *
         * Note: This uses the X of tag 10 in the official layout.
         */
        public static final double oppAllianceZone =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
    }

    /**
     * Convenient Y positions for important horizontal (field-widthwise) lines.
     *
     * Horizontal lines are represented as Y offsets in field coordinates.
     *
     * Convention: - "Start" and "End" are left-to-right from the perspective of the alliance
     * station (blue).
     */
    public static class LinesHorizontal {
        /** Field centerline Y (half the field width). */
        public static final double center = fieldWidth / 2.0;

        // Right of hub (toward lower Y in WPILib field coordinates)

        /** Y at the near edge of the right bump (closest to hub). */
        public static final double rightBumpStart = Hub.nearRightCorner.getY();

        /** Y at the far edge of the right bump (away from hub). */
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;

        /**
         * Y at the start of the open area adjacent to the right trench.
         *
         * Includes a small spacing from the bump.
         */
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);

        /** Y at the end of the right trench open segment (field boundary). */
        public static final double rightTrenchOpenEnd = 0;

        // Left of hub (toward higher Y in WPILib field coordinates)

        /** Y at the near edge of the left bump (closest to hub). */
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();

        /** Y at the far edge of the left bump (away from hub). */
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;

        /**
         * Y at the end of the open area adjacent to the left trench.
         *
         * Includes a small spacing from the bump.
         */
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);

        /** Y at the start of the left trench open segment (field boundary). */
        public static final double leftTrenchOpenStart = fieldWidth;
    }

    /**
     * Hub geometry and reference points (center, corners, and faces).
     *
     * Many points are defined using the AprilTag poses to remain consistent with the official field
     * layout.
     */
    public static class Hub {

        // Dimensions

        /** Outer hub diameter/width in meters. */
        public static final double width = Units.inchesToMeters(47.0);

        /**
         * Outer hub height in meters (includes the catcher/top feature).
         */
        public static final double height = Units.inchesToMeters(72.0);

        /** Inner opening diameter/width in meters. */
        public static final double innerWidth = Units.inchesToMeters(41.7);

        /** Inner opening height in meters. */
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Relevant reference points on alliance side

        /**
         * Center point of the top of the hub on the alliance side.
         *
         * Units: meters.
         */
        public static final Translation3d topCenterPoint = new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0, height);

        /**
         * Center point of the inner opening on the alliance side.
         *
         * Units: meters.
         */
        public static final Translation3d innerCenterPoint = new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0, innerHeight);

        /** Alliance-side hub corner closest to the alliance wall, left side. */
        public static final Translation2d nearLeftCorner =
            new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);

        /** Alliance-side hub corner closest to the alliance wall, right side. */
        public static final Translation2d nearRightCorner =
            new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);

        /** Alliance-side hub far corner (away from alliance wall), left side. */
        public static final Translation2d farLeftCorner =
            new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);

        /** Alliance-side hub far corner (away from alliance wall), right side. */
        public static final Translation2d farRightCorner =
            new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Relevant reference points on the opposite side

        /**
         * Center point of the top of the hub on the opposing side.
         */
        public static final Translation3d oppTopCenterPoint = new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0, height);

        /** Opposing-side hub corner closest to the far wall, left side. */
        public static final Translation2d oppNearLeftCorner = new Translation2d(
            oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);

        /** Opposing-side hub corner closest to the far wall, right side. */
        public static final Translation2d oppNearRightCorner = new Translation2d(
            oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);

        /** Opposing-side hub far corner (toward field center), left side. */
        public static final Translation2d oppFarLeftCorner = new Translation2d(
            oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);

        /** Opposing-side hub far corner (toward field center), right side. */
        public static final Translation2d oppFarRightCorner = new Translation2d(
            oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Hub faces

        /**
         * Pose of the hub "near" face (alliance side).
         *
         * Derived from AprilTag 26 pose.
         */
        public static final Pose2d nearFace =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();

        /**
         * Pose of the hub "far" face (toward opposing side).
         *
         * Derived from AprilTag 20 pose.
         */
        public static final Pose2d farFace =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();

        /**
         * Pose of the hub right face (lower Y direction).
         *
         * Derived from AprilTag 18 pose.
         */
        public static final Pose2d rightFace =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();

        /**
         * Pose of the hub left face (higher Y direction).
         *
         * Derived from AprilTag 21 pose.
         */
        public static final Pose2d leftFace =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
    }

    /**
     * Geometry for the left bump and common reference points.
     *
     * "Left" is defined from the perspective of the blue alliance station.
     */
    public static class LeftBump {

        /** Left bump width along the field Y axis (meters). */
        public static final double width = Units.inchesToMeters(73.0);

        /** Left bump height above the carpet (meters). */
        public static final double height = Units.inchesToMeters(6.513);

        /** Left bump depth along the field X axis (meters). */
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side

        /** Alliance-side left bump near-left corner point. */
        public static final Translation2d nearLeftCorner =
            new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));

        /** Alliance-side left bump near-right corner point (shared with hub). */
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;

        /** Alliance-side left bump far-left corner point. */
        public static final Translation2d farLeftCorner =
            new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));

        /** Alliance-side left bump far-right corner point (shared with hub). */
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side

        /** Opposing-side left bump near-left corner point. */
        public static final Translation2d oppNearLeftCorner =
            new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));

        /** Opposing-side left bump near-right corner point (shared with hub). */
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;

        /** Opposing-side left bump far-left corner point. */
        public static final Translation2d oppFarLeftCorner =
            new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));

        /** Opposing-side left bump far-right corner point (shared with hub). */
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /**
     * Geometry for the right bump and common reference points.
     *
     * "Right" is defined from the perspective of the blue alliance station.
     */
    public static class RightBump {
        /** Right bump width along the field Y axis (meters). */
        public static final double width = Units.inchesToMeters(73.0);

        /** Right bump height above the carpet (meters). */
        public static final double height = Units.inchesToMeters(6.513);

        /** Right bump depth along the field X axis (meters). */
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side

        /** Alliance-side right bump near-left corner point. */
        public static final Translation2d nearLeftCorner =
            new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));

        /** Alliance-side right bump near-right corner point (shared with hub). */
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;

        /** Alliance-side right bump far-left corner point. */
        public static final Translation2d farLeftCorner =
            new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));

        /** Alliance-side right bump far-right corner point (shared with hub). */
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side

        /** Opposing-side right bump near-left corner point. */
        public static final Translation2d oppNearLeftCorner =
            new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));

        /** Opposing-side right bump near-right corner point (shared with hub). */
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;

        /** Opposing-side right bump far-left corner point. */
        public static final Translation2d oppFarLeftCorner =
            new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));

        /** Opposing-side right bump far-right corner point (shared with hub). */
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /**
     * Left trench geometry and key opening reference points.
     *
     * This primarily provides 3D points useful for vision targeting to the trench opening.
     */
    public static class LeftTrench {
        /** Trench width (meters). */
        public static final double width = Units.inchesToMeters(65.65);

        /** Trench depth (meters). */
        public static final double depth = Units.inchesToMeters(47.0);

        /** Trench height (meters). */
        public static final double height = Units.inchesToMeters(40.25);

        /** Width of the trench opening (meters). */
        public static final double openingWidth = Units.inchesToMeters(50.34);

        /** Height of the trench opening (meters). */
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side

        /** Alliance-side opening top-left corner point (3D). */
        public static final Translation3d openingTopLeft =
            new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);

        /** Alliance-side opening top-right corner point (3D). */
        public static final Translation3d openingTopRight =
            new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

        // Relevant reference points on opposing side

        /** Opposing-side opening top-left corner point (3D). */
        public static final Translation3d oppOpeningTopLeft =
            new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);

        /** Opposing-side opening top-right corner point (3D). */
        public static final Translation3d oppOpeningTopRight =
            new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
    }

    /**
     * Right trench geometry and key opening reference points.
     *
     * This primarily provides 3D points useful for vision targeting to the trench opening.
     */
    public static class RightTrench {
        /** Trench width (meters). */
        public static final double width = Units.inchesToMeters(65.65);

        /** Trench depth (meters). */
        public static final double depth = Units.inchesToMeters(47.0);

        /** Trench height (meters). */
        public static final double height = Units.inchesToMeters(40.25);

        /** Width of the trench opening (meters). */
        public static final double openingWidth = Units.inchesToMeters(50.34);

        /** Height of the trench opening (meters). */
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side

        /** Alliance-side opening top-left corner point (3D). */
        public static final Translation3d openingTopLeft =
            new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);

        /** Alliance-side opening top-right corner point (3D). */
        public static final Translation3d openingTopRight =
            new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

        // Relevant reference points on opposing side

        /** Opposing-side opening top-left corner point (3D). */
        public static final Translation3d oppOpeningTopLeft =
            new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);

        /** Opposing-side opening top-right corner point (3D). */
        public static final Translation3d oppOpeningTopRight =
            new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
    }

    /**
     * Tower geometry and reference points (center and uprights).
     *
     * Useful for alignment targets and autonomous placement.
     */
    public static class Tower {
        /** Tower width (meters). */
        public static final double width = Units.inchesToMeters(49.25);

        /** Tower depth (meters). */
        public static final double depth = Units.inchesToMeters(45.0);

        /** Tower height (meters). */
        public static final double height = Units.inchesToMeters(78.25);

        /** Width of the inner opening (meters). */
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);

        /**
         * X coordinate of the tower front face in field coordinates (meters).
         */
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        /** Height of the uprights (meters). */
        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor

        /** Low rung height above carpet (meters). */
        public static final double lowRungHeight = Units.inchesToMeters(27.0);

        /** Mid rung height above carpet (meters). */
        public static final double midRungHeight = Units.inchesToMeters(45.0);

        /** High rung height above carpet (meters). */
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side

        /**
         * Tower center point on the alliance side (2D).
         *
         * Y is derived from AprilTag 31.
         */
        public static final Translation2d centerPoint = new Translation2d(frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());

        /**
         * Left upright position on the alliance side (2D).
         *
         * The extra 0.75 inch accounts for physical offset/clearance.
         */
        public static final Translation2d leftUpright = new Translation2d(frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                + innerOpeningWidth / 2 + Units.inchesToMeters(0.75));

        /**
         * Right upright position on the alliance side (2D).
         *
         * The extra 0.75 inch accounts for physical offset/clearance.
         */
        public static final Translation2d rightUpright = new Translation2d(frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                - innerOpeningWidth / 2 - Units.inchesToMeters(0.75));

        // Relevant reference points on opposing side

        /**
         * Tower center point on the opposing side (2D).
         *
         * X is mirrored about the field length. Y is derived from AprilTag 15.
         */
        public static final Translation2d oppCenterPoint =
            new Translation2d(fieldLength - frontFaceX,
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());

        /** Left upright position on the opposing side (2D). */
        public static final Translation2d oppLeftUpright =
            new Translation2d(fieldLength - frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                    + innerOpeningWidth / 2 + Units.inchesToMeters(0.75));

        /** Right upright position on the opposing side (2D). */
        public static final Translation2d oppRightUpright =
            new Translation2d(fieldLength - frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                    - innerOpeningWidth / 2 - Units.inchesToMeters(0.75));
    }

    /**
     * Depot geometry and reference points on the alliance side.
     */
    public static class Depot {
        /** Depot width (meters). */
        public static final double width = Units.inchesToMeters(42.0);

        /** Depot depth (meters). */
        public static final double depth = Units.inchesToMeters(27.0);

        /** Depot height (meters). */
        public static final double height = Units.inchesToMeters(1.125);

        /** Lateral Y distance from the field centerline to the depot center. */
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side

        /** Depot center point (3D). */
        public static final Translation3d depotCenter =
            new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);

        /** Depot left corner point (3D). */
        public static final Translation3d leftCorner =
            new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);

        /** Depot right corner point (3D). */
        public static final Translation3d rightCorner =
            new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    /**
     * Outpost geometry and reference points on the alliance side.
     */
    public static class Outpost {
        /** Outpost width (meters). */
        public static final double width = Units.inchesToMeters(31.8);

        /** Height of the opening bottom from the floor (meters). */
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);

        /** Outpost height (meters). */
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side

        /**
         * Outpost center point (2D).
         *
         * Y is derived from AprilTag 29. X is at the field wall.
         */
        public static final Translation2d centerPoint = new Translation2d(0,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
    }

    /**
     * Identifies which set of field-measurement JSONs to load.
     *
     * The {@link #jsonFolder} maps to a subdirectory under deploy/apriltags/.
     */
    @RequiredArgsConstructor
    public enum FieldType {
        /** Field built from AndyMark elements. */
        ANDYMARK("andymark"),
        /** Field built from welded elements. */
        WELDED("welded");

        /**
         * Deploy folder name containing JSON layouts for this field type.
         */
        @Getter
        private final String jsonFolder;
    }

    /**
     * Available AprilTag layouts for this project.
     *
     * Layouts are loaded lazily from JSON on first access and cached for subsequent calls.
     */
    public enum AprilTagLayoutType {
        /** Official season layout. */
        OFFICIAL("2026-official"),
        /** Empty layout intended for tests/simulation without tags. */
        NONE("2026-none");

        /** Base filename (without .json). */
        private final String name;

        /**
         * Cached layout instance.
         *
         * Marked volatile to support safe double-checked locking initialization.
         */
        private volatile AprilTagFieldLayout layout;

        /**
         * Cached JSON string of the loaded layout, useful for logging/dashboard.
         */
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        /**
         * Loads (if needed) and returns the {@link AprilTagFieldLayout} for this layout type.
         *
         * Loading behavior: - If {@code Constants.disableHAL} is true, loads from the project
         * source deploy path (useful in unit tests). - Otherwise loads from the robot deploy
         * directory.
         *
         * The loaded layout is cached after the first call.
         *
         * @return the loaded {@link AprilTagFieldLayout}
         * @throws RuntimeException if the JSON cannot be read/parsed
         */
        public AprilTagFieldLayout getLayout() {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        try {
                            Path p = Constants.disableHAL
                                ? Path.of("src", "main", "deploy", "apriltags",
                                    fieldType.getJsonFolder(), name + ".json")
                                : Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags",
                                    fieldType.getJsonFolder(), name + ".json");
                            layout = new AprilTagFieldLayout(p);
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
            }
            return layout;
        }

        /**
         * Returns a JSON string representation of the loaded layout.
         *
         * If the layout has not been loaded yet, this method will load it first.
         *
         * @return JSON string for the layout
         */
        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }
}
