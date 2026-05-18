package frc.gen;

import java.io.IOException;
import javax.swing.SwingUtilities;
import edu.wpi.first.cscore.OpenCvLoader;

public class TestApp {

    public static void main(String[] argv) {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }

        SwingUtilities.invokeLater(() -> {
            FrameProvider provider = new DummyFrameProvider(30 * 15, 1920, 1080);
            new VideoViewer(provider);
        });
    }

}
