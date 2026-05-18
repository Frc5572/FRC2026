package frc.gen;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Polygon;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import javax.swing.JPanel;

public class TimelinePanel extends JPanel {

    private int totalFrames;
    private int currentFrame;
    private final List<Integer> markers = new ArrayList<>();

    private boolean draggingPlayhead = false;
    private TimelineListener listener;

    public interface TimelineListener {
        void onSeekRequested(int frame);
    }

    public TimelinePanel(int totalFrames) {
        this.totalFrames = Math.max(1, totalFrames);

        setPreferredSize(new Dimension(800, 60));
        setBackground(Color.DARK_GRAY);

        MouseAdapter mouseHandler = new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                int frame = xToFrame(e.getX());
                currentFrame = frame;
                if (listener != null)
                    listener.onSeekRequested(frame);
                draggingPlayhead = true;
                repaint();
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                if (draggingPlayhead) {
                    int frame = xToFrame(e.getX());
                    currentFrame = frame;
                    if (listener != null)
                        listener.onSeekRequested(frame);
                    repaint();
                }
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                draggingPlayhead = false;
            }

            @Override
            public void mouseClicked(MouseEvent e) {
                // Left click already handled via mousePressed
                // Right-click could be used for context menu if desired
            }
        };
        addMouseListener(mouseHandler);
        addMouseMotionListener(mouseHandler);
    }

    public void setTimelineListener(TimelineListener l) {
        this.listener = l;
    }

    public void setTotalFrames(int totalFrames) {
        this.totalFrames = Math.max(1, totalFrames);
        repaint();
    }

    public void setCurrentFrame(int frame) {
        this.currentFrame = Math.max(0, Math.min(totalFrames - 1, frame));
        repaint();
    }

    public int getCurrentFrame() {
        return currentFrame;
    }

    public void addMarkerAtCurrentFrame() {
        if (!markers.contains(currentFrame)) {
            markers.add(currentFrame);
            Collections.sort(markers);
            repaint();
        }
    }

    public List<Integer> getMarkers() {
        return Collections.unmodifiableList(markers);
    }

    private int xToFrame(int x) {
        int w = getWidth();
        x = Math.max(0, Math.min(w - 1, x));
        double ratio = x / (double) (w - 1);
        return (int) Math.round(ratio * (totalFrames - 1));
    }

    private int frameToX(int frame) {
        int w = getWidth();
        if (totalFrames <= 1)
            return 0;
        double ratio = frame / (double) (totalFrames - 1);
        return (int) Math.round(ratio * (w - 1));
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        int w = getWidth();
        int h = getHeight();

        // Timeline base
        int lineY = h / 2;
        g.setColor(Color.GRAY);
        g.fillRect(0, lineY - 3, w, 6);

        // Draw markers
        g.setColor(Color.ORANGE);
        int markerHeight = 18;
        for (int frame : markers) {
            int x = frameToX(frame);
            int top = lineY - markerHeight - 2;
            int baseWidth = 8;
            Polygon triangle = new Polygon();
            triangle.addPoint(x, top);
            triangle.addPoint(x - baseWidth / 2, lineY - 3);
            triangle.addPoint(x + baseWidth / 2, lineY - 3);
            g.fillPolygon(triangle);
        }

        // Draw playhead
        int x = frameToX(currentFrame);
        g.setColor(Color.RED);
        g.drawLine(x, 0, x, h);

        // Current frame text
        g.setColor(Color.WHITE);
        String text = "Frame: " + currentFrame + " / " + (totalFrames - 1);
        g.drawString(text, 10, 15);
    }

}
