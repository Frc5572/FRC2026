// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.image.BufferedImage;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Base64;
import java.util.function.ToDoubleFunction;
import javax.imageio.ImageIO;
import frc.robot.ShotData.ShotEntry;
import frc.robot.math.interp2d.Interp2d;
import frc.robot.math.interp2d.MulAdd;
import frc.robot.util.Tuples.Tuple3;
import frc.robot.viz.DrawColorMap;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>
     * If you change your main robot class, change the parameter type.
     *
     * @param args String args
     */
    public static void main(String... args) {
        drawImages();
        // RobotBase.startRobot(Robot::new);
    }

    private static void drawImages() {
        drawImage("interp2d", ShotData.entries, "distance (ft)", ShotEntry::distanceFeet,
            "Flywheel Speed (rps)", ShotEntry::flywheelSpeedRps, "Hood Angle (deg)",
            ShotEntry::hoodAngleDeg, ShotData.mulAdd);
    }

    private static <T> void drawImage(String name, T[] data, String xName,
        ToDoubleFunction<T> xFunc, String yName, ToDoubleFunction<T> yFunc, String zName,
        ToDoubleFunction<T> zFunc, MulAdd<T> mulAdd) {
        double xMin = Arrays.stream(data).mapToDouble(xFunc).min().getAsDouble();
        double xMax = Arrays.stream(data).mapToDouble(xFunc).max().getAsDouble();
        double yMin = Arrays.stream(data).mapToDouble(yFunc).min().getAsDouble();
        double yMax = Arrays.stream(data).mapToDouble(yFunc).max().getAsDouble();

        Interp2d<T> interp = new Interp2d<T>(data, mulAdd, xFunc, yFunc);
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

        for (var item : data) {
            double x = xFunc.applyAsDouble(item);
            double y = yFunc.applyAsDouble(item);
            double u = (x - xMin) / (xMax - xMin);
            double v = 1.0 - (y - yMin) / (yMax - yMin);
            builder.append(circleSvg.replaceAll("\\{cx\\}", (u * 200.0 + 60.0) + "")
                .replaceAll("\\{cy\\}", (v * 200.0 + 10.0) + ""));
        }

        DecimalFormat df = new DecimalFormat("#.##");

        try (var writer = new BufferedWriter(new FileWriter(name + ".svg"))) {
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
                <text transform="translate(260, 215) rotate(90)" alignment-baseline="hanging" font-size="12px" stroke="none"
                fill="black">{xMax}</text>
                <text transform="translate(60, 215) rotate(90)" font-size="12px" stroke="none" fill="black">{xMin}</text>

                <image x="270" y="10" width="20" height="200" xlink:href="{b64Key}" />
                <text alignment-baseline="hanging" transform="translate(285, 10)" font-size="12px" stroke="none" fill="black">{zMax}</text>
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
