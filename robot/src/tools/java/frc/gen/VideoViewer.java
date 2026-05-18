package frc.gen;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.event.KeyEvent;
import java.awt.image.BufferedImage;
import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.KeyStroke;
import javax.swing.Timer;

public class VideoViewer extends JFrame {

    private final FrameProvider frameProvider;
    private final PoseEstimator poseEstimator;
    private final VideoPanel videoPanel;
    private final TimelinePanel timelinePanel;
    private final PoseResultPanel poseResultPanel;
    private final JButton playButton;
    private final JButton pauseButton;
    private final JButton addMarkerButton;

    private Timer playbackTimer;
    private int currentFrame = 0;
    private boolean playing = false;
    private int fps = 30; // playback speed

    public VideoViewer(FrameProvider provider, PoseEstimator poseEstimator) {
        this.frameProvider = provider;
        this.poseEstimator = poseEstimator;

        videoPanel = new VideoPanel();
        timelinePanel = new TimelinePanel(provider.getTotalFrames());
        poseResultPanel = new PoseResultPanel();

        playButton = new JButton("Play");
        pauseButton = new JButton("Pause");
        addMarkerButton = new JButton("Add Marker");

        initUI();
        initPlayback();
        loadFrame(0);
    }

    private void initUI() {
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new BorderLayout());

        add(videoPanel, BorderLayout.CENTER);
        add(poseResultPanel, BorderLayout.EAST);

        JPanel bottomPanel = new JPanel(new BorderLayout());
        bottomPanel.add(timelinePanel, BorderLayout.CENTER);

        JPanel controls = new JPanel(new FlowLayout(FlowLayout.LEFT));
        controls.add(playButton);
        controls.add(pauseButton);
        controls.add(addMarkerButton);
        bottomPanel.add(controls, BorderLayout.NORTH);

        add(bottomPanel, BorderLayout.SOUTH);

        // Listeners
        playButton.addActionListener(e -> startPlayback());
        pauseButton.addActionListener(e -> pausePlayback());
        addMarkerButton.addActionListener(e -> {
            timelinePanel.addMarkerAtCurrentFrame();
            // You might also persist marker info elsewhere here
        });

        timelinePanel.setTimelineListener(frame -> {
            currentFrame = frame;
            loadFrame(currentFrame);
        });

        // Key bindings on the root pane fire regardless of which component has focus
        var inputMap = getRootPane().getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
        var actionMap = getRootPane().getActionMap();

        inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, 0), "step-back");
        inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, 0), "step-forward");
        inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, KeyEvent.CTRL_DOWN_MASK), "step-back-10");
        inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, KeyEvent.CTRL_DOWN_MASK), "step-forward-10");

        actionMap.put("step-back", new AbstractAction() {
            public void actionPerformed(java.awt.event.ActionEvent e) { pausePlayback(); seekRelative(-1); }
        });
        actionMap.put("step-forward", new AbstractAction() {
            public void actionPerformed(java.awt.event.ActionEvent e) { pausePlayback(); seekRelative(1); }
        });
        actionMap.put("step-back-10", new AbstractAction() {
            public void actionPerformed(java.awt.event.ActionEvent e) { pausePlayback(); seekRelative(-10); }
        });
        actionMap.put("step-forward-10", new AbstractAction() {
            public void actionPerformed(java.awt.event.ActionEvent e) { pausePlayback(); seekRelative(10); }
        });

        pack();
        setLocationRelativeTo(null);
        setVisible(true);
    }

    private void initPlayback() {
        int delay = 1000 / fps;
        playbackTimer = new Timer(delay, e -> {
            if (!playing)
                return;
            currentFrame++;
            if (currentFrame >= frameProvider.getTotalFrames()) {
                currentFrame = frameProvider.getTotalFrames() - 1;
                pausePlayback();
            }
            timelinePanel.setCurrentFrame(currentFrame);
            loadFrame(currentFrame);
        });
    }

    private void startPlayback() {
        if (!playing) {
            playing = true;
            playbackTimer.start();
        }
    }

    private void pausePlayback() {
        playing = false;
        playbackTimer.stop();
    }

    private void loadFrame(int frameIndex) {
        BufferedImage img = frameProvider.getFrame(frameIndex);
        videoPanel.setFrame(img);
        timelinePanel.setCurrentFrame(frameIndex);
        poseResultPanel.update(poseEstimator.estimate(img).orElse(null));
    }

    private void seekRelative(int delta) {
        int newFrame = currentFrame + delta;
        newFrame = Math.max(0, Math.min(frameProvider.getTotalFrames() - 1, newFrame));
        currentFrame = newFrame;
        loadFrame(newFrame);
    }

}
