package frc.robot.util.tunable;

import java.awt.image.BufferedImage;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Base64;
import java.util.EnumSet;
import java.util.function.ToDoubleFunction;
import javax.imageio.ImageIO;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ShotData;
import frc.robot.ShotData.ShotEntry;
import frc.robot.math.interp2d.Interp2d;
import frc.robot.util.Tuples.Tuple3;
import frc.robot.viz.DrawColorMap;

/** Utility tunable for testing out different shot parameters. */
public class ShotDataHelper implements Tunable {

    /** Hood angle in degrees */
    public double hoodAngle = 5.0;
    /** Flywheel speed in rotations per second */
    public double flywheelSpeed = 60.0;
    /** Distance from target in feet */
    public double distanceFromTarget = 10.0;

    private final DoublePublisher timeLastGenerated;

    /** Create new helper. */
    public ShotDataHelper() {
        var nt = NetworkTableInstance.getDefault();
        timeLastGenerated = nt.getDoubleTopic("/ShotDataHelper/TimeLastGenerated").publish();
        Tunable.setupTunable("/ShotDataHelper", this, ShotDataHelper.class, () -> {
        });
        var topic = nt.getDoubleArrayTopic("/ShotDataHelper/Regenerate");
        double[] initData = new double[ShotData.entries.length * 4];
        for (int i = 0; i < ShotData.entries.length; i++) {
            initData[i * 4 + 0] = ShotData.entries[i].distanceFeet();
            initData[i * 4 + 1] = ShotData.entries[i].flywheelSpeedRps();
            initData[i * 4 + 2] = ShotData.entries[i].hoodAngleDeg();
            initData[i * 4 + 3] = ShotData.entries[i].timeOfFlight();
        }
        generateImages(initData);
        topic.publish().accept(initData);
        nt.addListener(topic.subscribe(new double[0]),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                double[] data = ev.valueData.value.getDoubleArray();
                generateImages(data);
            });
    }

    private void generateImages(double[] data) {
        ShotData.ShotEntry[] entries = new ShotEntry[data.length / 4];
        for (int i = 0; i < data.length; i += 4) {
            entries[i / 4] = new ShotEntry(data[i + 0], data[i + 1], data[i + 2], data[i + 3]);
        }
        Interp2d<ShotData.ShotEntry> interp = new Interp2d<>(entries, ShotData.mulAdd,
            ShotData.ShotEntry::distanceFeet, ShotData.ShotEntry::flywheelSpeedRps);

        drawImage("hood_angle", interp, "distance (ft)", ShotEntry::distanceFeet,
            "Flywheel Speed (rps)", ShotEntry::flywheelSpeedRps, "Hood Angle (deg)",
            ShotEntry::hoodAngleDeg);
        drawImage("time_of_flight", interp, "distance (ft)", ShotEntry::distanceFeet,
            "Flywheel Speed (rps)", ShotEntry::flywheelSpeedRps, "Time of Flight (s)",
            ShotEntry::timeOfFlight);
        drawImage("horizontal_velocity", interp, "distance (ft)", ShotEntry::distanceFeet,
            "Flywheel Speed (rps)", ShotEntry::flywheelSpeedRps, "Horizontal Velocity (m/s)",
            ShotEntry::horizontalVelocity);
        drawImage("vertical_velocity", interp, "distance (ft)", ShotEntry::distanceFeet,
            "Flywheel Speed (rps)", ShotEntry::flywheelSpeedRps, "Vertical Velocity (m/s)",
            ShotEntry::verticalVelocity);

        timeLastGenerated.accept(Timer.getFPGATimestamp());
    }

    private static <T> void drawImage(String name, Interp2d<T> interp, String xName,
        ToDoubleFunction<T> xFunc, String yName, ToDoubleFunction<T> yFunc, String zName,
        ToDoubleFunction<T> zFunc) {
        double xMin = Arrays.stream(interp.data).mapToDouble(xFunc).min().getAsDouble();
        double xMax = Arrays.stream(interp.data).mapToDouble(xFunc).max().getAsDouble();
        double yMin = Arrays.stream(interp.data).mapToDouble(yFunc).min().getAsDouble();
        double yMax = Arrays.stream(interp.data).mapToDouble(yFunc).max().getAsDouble();

        Tuple3<BufferedImage, Double, Double> res = null;
        BufferedImage key = null;
        try {
            res = DrawColorMap.draw(x -> zFunc.applyAsDouble(interp.query(x).value()), xMin, xMax,
                yMin, yMax);
            key = DrawColorMap.key();
        } catch (IOException e) {
            e.printStackTrace();
        }
        String b64Res = imageToBase64(res._0());
        String b64Key = imageToBase64(key);
        double zMin = res._1();
        double zMax = res._2();

        StringBuilder builder = new StringBuilder();

        for (var item : interp.data) {
            double x = xFunc.applyAsDouble(item);
            double y = yFunc.applyAsDouble(item);
            double u = (x - xMin) / (xMax - xMin);
            double v = 1.0 - (y - yMin) / (yMax - yMin);
            builder.append(circleSvg.replaceAll("\\{cx\\}", (u * 200.0 + 60.0) + "")
                .replaceAll("\\{cy\\}", (v * 200.0 + 10.0) + ""));
        }

        DecimalFormat df = new DecimalFormat("#.##");

        try (var writer = new BufferedWriter(new FileWriter(
            new File(Filesystem.getDeployDirectory(), "shotdata/" + name + ".svg")))) {
            writer.write(svgTemplate.replaceAll("\\{b64Res\\}", b64Res)
                .replaceAll("\\{b64Key\\}", b64Key).replaceAll("\\{xMin\\}", df.format(xMin))
                .replaceAll("\\{xMax\\}", df.format(xMax)).replaceAll("\\{yMin\\}", df.format(yMin))
                .replaceAll("\\{yMax\\}", df.format(yMax)).replaceAll("\\{zMin\\}", df.format(zMin))
                .replaceAll("\\{zMax\\}", df.format(zMax)).replaceAll("\\{xName\\}", xName)
                .replaceAll("\\{yName\\}", yName).replaceAll("\\{zName\\}", zName)
                .replaceAll("\\{circles\\}", builder.toString()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static String imageToBase64(BufferedImage image) {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        try {
            ImageIO.write(image, "png", baos);
        } catch (IOException e) {
            e.printStackTrace();
        }
        String base64 = Base64.getEncoder().encodeToString(baos.toByteArray());
        return "data:image/png;base64," + base64;
    }

    private static final String svgTemplate =
        """
            <svg version="1.1" baseProfile="tiny"
            width="100%" height="100%" viewBox="0 0 350 250"
            xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink">
                <rect width="100%" height="100%" fill="white" />
                <image x="60" y="10" width="200" height="200" xlink:href="{b64Res}" />
                <text text-anchor="end" alignment-baseline="hanging" transform="translate(55, 10)" font-size="12px" stroke="none"
                fill="black">{yMax}</text>
                <text text-anchor="end" transform="translate(55, 210)" font-size="12px" stroke="none" fill="black">{yMin}</text>
                <text text-anchor="end" transform="translate(260, 215) rotate(-60)" font-size="12px" stroke="none"
                fill="black">{xMax}</text>
                <text text-anchor="end" transform="translate(60, 215) rotate(-60)" alignment-baseline="hanging"
                font-size="12px" stroke="none" fill="black">{xMin}</text>

                <image x="270" y="10" width="20" height="200" xlink:href="{b64Key}" />
                <text alignment-baseline="hanging" transform="translate(285, 10)" font-size="12px" stroke="none"
                fill="black">{zMax}</text>
                <text transform="translate(285, 210)" font-size="12px" stroke="none" fill="black">{zMin}</text>

                <text transform="translate(160, 225)" text-anchor="middle" alignment-baseline="hanging" font-size="12px"
                stroke="none" fill="black">{xName}</text>
                <text transform="translate(40, 110) rotate(-90)" text-anchor="middle" font-size="12px" stroke="none"
                fill="black">{yName}</text>

                <text transform="translate(305, 110) rotate(90)" text-anchor="middle" font-size="12px" stroke="none"
                fill="black">{zName}</text>

                {circles}
            </svg>""";

    private static final String circleSvg =
        "<circle cx=\"{cx}\" cy=\"{cy}\" r=\"2\" stroke=\"black\" stroke-width=\"1\" fill=\"none\" />";

}
