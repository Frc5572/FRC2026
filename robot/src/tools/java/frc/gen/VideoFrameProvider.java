package frc.gen;

import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.util.LinkedHashMap;
import java.util.Map;
import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FrameGrabber.Exception;
import org.bytedeco.javacv.Java2DFrameConverter;

public class VideoFrameProvider implements FrameProvider {

    private static final int CACHE_SIZE = 64;

    private final FFmpegFrameGrabber grabber;
    private final Java2DFrameConverter converter = new Java2DFrameConverter();
    private final int numFrames;
    private final LinkedHashMap<Integer, BufferedImage> cache = new LinkedHashMap<>(CACHE_SIZE, 0.75f, true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<Integer, BufferedImage> eldest) {
            return size() > CACHE_SIZE;
        }
    };

    public VideoFrameProvider(String path) {
        grabber = new FFmpegFrameGrabber(path);
        try {
            grabber.start();
            numFrames = grabber.getLengthInFrames();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public BufferedImage getFrame(int frameIndex) {
        BufferedImage cached = cache.get(frameIndex);
        if (cached != null) return cached;

        try {
            grabber.setFrameNumber(frameIndex);
            Frame frame;
            do {
                frame = grabber.grabImage();
                if (frame == null) {
                    throw new NullPointerException("frame is null before reaching target");
                }
            } while (grabber.getFrameNumber() < frameIndex);

            var image = converter.convert(frame);
            if (image == null) {
                throw new NullPointerException("image is null");
            }
            BufferedImage result = applyRotation(image, -grabber.getDisplayRotation());
            cache.put(frameIndex, result);
            return result;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private BufferedImage applyRotation(BufferedImage image, double degrees) {
        if (degrees == 0) return image;
        double rad = Math.toRadians(degrees);
        double sin = Math.abs(Math.sin(rad));
        double cos = Math.abs(Math.cos(rad));
        int w = image.getWidth();
        int h = image.getHeight();
        int newW = (int) Math.round(w * cos + h * sin);
        int newH = (int) Math.round(w * sin + h * cos);
        BufferedImage out = new BufferedImage(newW, newH, image.getType());
        Graphics2D g = out.createGraphics();
        AffineTransform t = AffineTransform.getTranslateInstance(newW / 2.0, newH / 2.0);
        t.rotate(rad);
        t.translate(-w / 2.0, -h / 2.0);
        g.drawImage(image, t, null);
        g.dispose();
        return out;
    }

    @Override
    public int getTotalFrames() {
        return numFrames;
    }

}
