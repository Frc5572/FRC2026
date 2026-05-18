package frc.gen;

import java.awt.image.BufferedImage;

public interface FrameProvider {
    BufferedImage getFrame(int frameIndex);

    int getTotalFrames();
}
