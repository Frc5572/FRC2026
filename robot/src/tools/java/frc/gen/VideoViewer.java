package frc.gen;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.Timer;

public class VideoViewer extends JFrame {

    private final FrameProvider frameProvider;
    private final VideoPanel videoPanel;
    private final TimelinePanel timelinePanel;
    private final JButton playButton;
    private final JButton pauseButton;
    private final JButton addMarkerButton;

    private Timer playbackTimer;
    private int currentFrame = 0;
    private boolean playing = false;
    private int fps = 30; // playback speed

    public VideoViewer(FrameProvider provider) {
        this.frameProvider = provider;

        videoPanel = new VideoPanel();
        timelinePanel = new TimelinePanel(provider.getTotalFrames());

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

        add(new JScrollPane(videoPanel), BorderLayout.CENTER);

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

        // Keyboard listener for frame stepping
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                int code = e.getKeyCode();
                int step = e.isControlDown() ? 10 : 1; // Ctrl+arrow for bigger jumps

                if (code == KeyEvent.VK_LEFT) {
                    pausePlayback(); // often desirable to pause when scrubbing
                    seekRelative(-step);
                } else if (code == KeyEvent.VK_RIGHT) {
                    pausePlayback();
                    seekRelative(step);
                }
            }
        });

        // Make sure the frame receives key events
        setFocusable(true);
        addWindowListener(new WindowAdapter() {
            @Override
            public void windowOpened(WindowEvent e) {
                requestFocusInWindow();
            }
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
    }

    private void seekRelative(int delta) {
        int newFrame = currentFrame + delta;
        newFrame = Math.max(0, Math.min(frameProvider.getTotalFrames() - 1, newFrame));
        currentFrame = newFrame;
        loadFrame(newFrame);
    }

}
