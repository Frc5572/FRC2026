package frc.gen;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import javax.swing.JPanel;

public class VideoPanel extends JPanel {
    private BufferedImage currentFrame;

    public void setFrame(BufferedImage img) {
        this.currentFrame = img;
        repaint();
    }

    @Override
    public Dimension getPreferredSize() {
        if (currentFrame != null) {
            return new Dimension(currentFrame.getWidth(), currentFrame.getHeight());
        }
        return new Dimension(640, 360);
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (currentFrame != null) {
            int w = getWidth();
            int h = getHeight();
            // Fit frame while preserving aspect ratio
            double imgW = currentFrame.getWidth();
            double imgH = currentFrame.getHeight();
            double scale = Math.min(w / imgW, h / imgH);
            int drawW = (int) (imgW * scale);
            int drawH = (int) (imgH * scale);
            int x = (w - drawW) / 2;
            int y = (h - drawH) / 2;
            g.drawImage(currentFrame, x, y, drawW, drawH, null);
        }
    }

}
