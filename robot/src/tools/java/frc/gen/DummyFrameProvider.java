package frc.gen;

import java.awt.Color;
import java.awt.FontMetrics;
import java.awt.GradientPaint;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

public class DummyFrameProvider implements FrameProvider {
    private final int totalFrames;
    private final int width;
    private final int height;

    public DummyFrameProvider(int totalFrames, int width, int height) {
        this.totalFrames = totalFrames;
        this.width = width;
        this.height = height;
    }

    @Override
    public BufferedImage getFrame(int frameIndex) {
        frameIndex = Math.max(0, Math.min(totalFrames - 1, frameIndex));
        BufferedImage img = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = img.createGraphics();
        // Simple gradient that depends on frameIndex
        float t = frameIndex / (float) (totalFrames - 1);
        Color c1 = Color.getHSBColor(t, 0.8f, 0.8f);
        Color c2 = Color.getHSBColor(1 - t, 0.8f, 0.8f);
        GradientPaint gp = new GradientPaint(0, 0, c1, width, height, c2);
        g.setPaint(gp);
        g.fillRect(0, 0, width, height);

        // Draw frame number
        g.setColor(Color.WHITE);
        g.setFont(g.getFont().deriveFont(32f));
        String text = "Frame " + frameIndex;
        FontMetrics fm = g.getFontMetrics();
        int tx = (width - fm.stringWidth(text)) / 2;
        int ty = (height - fm.getHeight()) / 2 + fm.getAscent();
        g.drawString(text, tx, ty);
        g.dispose();
        return img;
    }

    @Override
    public int getTotalFrames() {
        return totalFrames;
    }
}
