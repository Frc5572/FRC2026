package frc.robot.viz;

import java.awt.image.BufferedImage;
import java.util.function.ToDoubleFunction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import frc.robot.math.interp2d.Range;

public class SdfDrawer {

    public static BufferedImage drawSdf(Range xRange, Range yRange,
        ToDoubleFunction<Translation2d> sdf, int width, int height) {
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_4BYTE_ABGR);

        for (int ix = 0; ix < width; ix++) {
            double x = xRange.min + xRange.range * ((double) ix / ((double) width - 1));
            for (int iy = 0; iy < height; iy++) {
                double y = yRange.min + yRange.range * ((double) iy / ((double) height - 1));
                double d = sdf.applyAsDouble(new Translation2d(x, y));

                // coloring from https://www.shadertoy.com/view/3ltSW2
                var col = (d > 0.0) ? vec3(0.9, 0.6, 0.3) : vec3(0.65, 0.85, 1.0);
                col = col.times(1.0 - Math.exp(-6.0 * Math.abs(d)));
                col = col.times(0.8 + 0.2 * Math.cos(30.0 * d));
                // col = mix(col, vec3(1.0), 1.0 - smoothstep(0.0, 0.01, Math.abs(d)));

                image.setRGB(ix, height - iy - 1, drawColor(vec4(col, 1.0)));
            }
        }

        return image;
    }

    private static int drawColor(Vector<N4> color) {
        int r = (int) Math.round(MathUtil.clamp(color.get(0), 0.0, 1.0) * 255.0);
        int g = (int) Math.round(MathUtil.clamp(color.get(1), 0.0, 1.0) * 255.0);
        int b = (int) Math.round(MathUtil.clamp(color.get(2), 0.0, 1.0) * 255.0);
        int a = (int) Math.round(MathUtil.clamp(color.get(3), 0.0, 1.0) * 255.0);
        return (a & 0xff) << 24 | (r & 0xff) << 16 | (g & 0xff) << 8 | (b & 0xff);
    }

    private static Vector<N3> vec3(double a) {
        return VecBuilder.fill(a, a, a);
    }

    private static Vector<N3> vec3(double a, double b, double c) {
        return VecBuilder.fill(a, b, c);
    }

    private static Vector<N4> vec4(Vector<N3> v, double a) {
        return VecBuilder.fill(v.get(0), v.get(1), v.get(2), a);
    }

    private static double smoothstep(double edge0, double edge1, double x) {
        var t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }

    private static double clamp(double v, double min, double max) {
        return MathUtil.clamp(v, min, max);
    }

    private static Vector<N3> mix(Vector<N3> x, Vector<N3> y, double a) {
        return x.times(1.0 - a).plus(y.times(a));
    }

}
