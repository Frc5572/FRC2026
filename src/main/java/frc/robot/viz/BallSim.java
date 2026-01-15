package frc.robot.viz;

import static edu.wpi.first.units.Units.Meters;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.FieldConstants;

public class BallSim {

    private final List<Ball> balls = new ArrayList<>();

    private BallCounter blue = new BallCounter(
        new Translation2d(FieldConstants.Trench.distanceFromAllianceWall.in(Meters) / 2.0,
            FieldConstants.fieldWidth.in(Meters) / 2.0),
        0);
    private BallCounter red = new BallCounter(new Translation2d(
        FieldConstants.fieldLength.in(Meters)
            - FieldConstants.Trench.distanceFromAllianceWall.in(Meters) / 2.0,
        FieldConstants.fieldWidth.in(Meters) / 2.0), 0);
    private BallCounter mid =
        new BallCounter(new Translation2d(FieldConstants.fieldLength.in(Meters) / 2.0,
            FieldConstants.fieldWidth.in(Meters) / 2.0), 0);


    public void periodic() {
        for (Ball ball : balls) {
            final int substeps = 5;
            for (int i = 0; i < substeps; i++) {
                ball.update(TimedRobot.kDefaultPeriod / substeps);
            }
            if (ball.position.getZ() < 0.0) {
                if (ball.position.getX() < FieldConstants.Trench.distanceFromAllianceWall.in(Meters)
                    + FieldConstants.Trench.width.in(Meters) / 2.0) {
                    blue.count++;
                } else if (ball.position.getX() > FieldConstants.fieldLength.in(Meters)
                    - (FieldConstants.Trench.distanceFromAllianceWall.in(Meters)
                        + FieldConstants.Trench.width.in(Meters) / 2.0)) {
                    red.count++;
                } else {
                    mid.count++;
                }
            }
        }

        balls.removeIf((ball) -> ball.position.getZ() < 0.0);

        Logger.recordOutput("Viz/Balls",
            balls.stream().map((ball) -> ball.position).toArray(Translation3d[]::new));

        blue.update();
        red.update();
        mid.update();

        Logger.recordOutput("Viz/BlueNumber", blue.buffer);
        Logger.recordOutput("Viz/RedNumber", red.buffer);
        Logger.recordOutput("Viz/MidNumber", mid.buffer);
    }

    public void spawnBall(Translation3d position, Translation3d velocity) {
        balls.add(new Ball(position, velocity));
    }

    private static final class Ball {
        public Translation3d position;
        public Translation3d velocity;

        public Ball(Translation3d position, Translation3d velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        public void update(double dt) {
            position = position.plus(velocity.times(dt));
            velocity = velocity.plus(new Translation3d(0, 0, -9.81).times(dt));
        }
    }

    private static final class BallCounter {
        public final Translation3d[] buffer =
            new Translation3d[NumberDrawing.numVertices * 3 + Circle.numVertices];
        private final MultiNumberDrawing number;
        private final Circle circle;
        public int count;

        public BallCounter(Translation2d center, int count) {
            this.count = count;
            number =
                new MultiNumberDrawing(3, new Pose2d(center, Rotation2d.kZero), 0.5, buffer, 0);
            circle = new Circle(center, buffer, NumberDrawing.numVertices * 3);
        }

        public void update() {
            number.update(count);
            circle.update(count / 20.0);
        }
    }

    private static final class Circle {
        private static final double aboveOrBelowFieldThreshold = 0.05;
        public static final int numVertices = 22;
        public static final int numShownVertices = 20;
        private final Translation3d[] buffer;
        private final int offset;
        private final Translation2d center;

        public Circle(Translation2d center, Translation3d[] buffer, int offset) {
            this.center = center;
            this.buffer = buffer;
            this.offset = offset;
        }

        public void update(double radius) {
            buffer[offset] = new Translation3d(center.plus(new Translation2d(radius, 0)))
                .minus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            buffer[offset + numVertices - 1] = buffer[offset];
            for (int i = 0; i < numShownVertices; i++) {
                buffer[offset + i + 1] = new Translation3d(center.plus(new Translation2d(radius,
                    Rotation2d.fromRotations((double) i / (double) (numShownVertices - 1)))))
                        .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            }
        }
    }

    private static final class MultiNumberDrawing {
        private final NumberDrawing[] numbers;

        public MultiNumberDrawing(int numNumbers, Pose2d start, double size, Translation3d[] buffer,
            int offset) {
            double width = (numNumbers - 1) * 1.3 + 1;
            numbers = new NumberDrawing[numNumbers];
            for (int i = 0; i < numNumbers; i++) {
                numbers[i] = new NumberDrawing(
                    new Pose2d(
                        start.getTranslation().plus(new Translation2d(-width / 4.0, -1.0 / 2.0))
                            .plus(new Translation2d(1.3, 0).times(i).times(size)
                                .rotateBy(start.getRotation())),
                        start.getRotation()),
                    size, buffer, offset + i * 18);
            }
        }

        public void update(int num) {
            for (int i = 0; i < numbers.length; i++) {
                numbers[numbers.length - i - 1].update(num);
                num /= 10;
            }
        }
    }

    private static final class NumberDrawing {
        private static final double aboveOrBelowFieldThreshold = 0.05;
        public static final int numVertices = 18;
        private final Translation3d[] buffer;
        private final int offset;

        private final Translation3d[] parts_off;
        private final Translation3d[] parts_on;

        public NumberDrawing(Pose2d pose, double size, Translation3d[] buffer, int offset) {
            this.buffer = buffer;
            this.offset = offset;

            this.parts_off = new Translation3d[16];
            this.parts_on = new Translation3d[16];
            Translation2d[] points = new Translation2d[6];
            for (int x = 0; x < 2; x++) {
                for (int y = 0; y < 3; y++) {
                    int index = x * 3 + y;
                    points[index] = pose.getTranslation().plus(
                        new Translation2d(x, 2.0 - y).times(size).rotateBy(pose.getRotation()));
                }
            }
            // Top
            parts_on[0] = new Translation3d(points[(0 * 3 + 0)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[1] = new Translation3d(points[(1 * 3 + 0)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            // Right Top
            parts_on[2] = new Translation3d(points[(1 * 3 + 0)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[3] = new Translation3d(points[(1 * 3 + 1)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            // Right Bottom
            parts_on[4] = new Translation3d(points[(1 * 3 + 1)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[5] = new Translation3d(points[(1 * 3 + 2)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            // Bottom
            parts_on[6] = new Translation3d(points[(1 * 3 + 2)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[7] = new Translation3d(points[(0 * 3 + 2)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            // Left Bottom
            parts_on[8] = new Translation3d(points[(0 * 3 + 2)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[9] = new Translation3d(points[(0 * 3 + 1)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            // Left Top
            parts_on[10] = new Translation3d(points[(0 * 3 + 1)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[11] = new Translation3d(points[(0 * 3 + 0)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            // Left Top Backtrack
            parts_on[12] = new Translation3d(points[(0 * 3 + 0)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[13] = new Translation3d(points[(0 * 3 + 1)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            // Middle
            parts_on[14] = new Translation3d(points[(0 * 3 + 1)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));
            parts_on[15] = new Translation3d(points[(1 * 3 + 1)])
                .plus(new Translation3d(0, 0, aboveOrBelowFieldThreshold));

            for (int i = 0; i < 16; i++) {
                parts_off[i] =
                    parts_on[i].minus(new Translation3d(0, 0, 2.0 * aboveOrBelowFieldThreshold));
            }

            buffer[offset] = parts_off[0];
            buffer[offset + 17] = parts_off[15];
        }

        public void update(int num) {
            num = num % 10;
            switch (num) {
                case 1 -> {
                    updateSegments(1, 2);
                }
                case 2 -> {
                    updateSegments(0, 1, 3, 4, 7);
                }
                case 3 -> {
                    updateSegments(0, 1, 2, 3, 7);
                }
                case 4 -> {
                    updateSegments(1, 2, 5, 7);
                }
                case 5 -> {
                    updateSegments(0, 2, 3, 5, 7);
                }
                case 6 -> {
                    updateSegments(0, 2, 3, 4, 5, 7);
                }
                case 7 -> {
                    updateSegments(0, 1, 2);
                }
                case 8 -> {
                    updateSegments(0, 1, 2, 3, 4, 5, 7);
                }
                case 9 -> {
                    updateSegments(0, 1, 2, 3, 5, 7);
                }
                default -> {
                    updateSegments(0, 1, 2, 3, 4, 5);
                }
            }
        }

        private void updateSegments(int... on) {
            for (int i = 0; i < 16; i++) {
                buffer[i + offset + 1] = parts_off[i];
            }
            for (int i : on) {
                buffer[offset + (2 * i) + 1] = parts_on[(2 * i) + 0];
                buffer[offset + (2 * i) + 2] = parts_on[(2 * i) + 1];
            }
        }
    }

}
