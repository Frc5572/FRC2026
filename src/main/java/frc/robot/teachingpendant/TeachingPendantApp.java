package frc.robot.teachingpendant;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.RenderingHints;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import javax.swing.BorderFactory;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JSpinner;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.JTree;
import javax.swing.SpinnerNumberModel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeModel;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;

/** Desktop application launched by {@code ./gradlew teachingPendant}. */
public final class TeachingPendantApp {
    private static final Path AUTOS = Path.of("src", "main", "deploy", "jrtp-autos");
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable manual = nt.getTable("/rosbots/TeachingPendant/Manual");
    private final NetworkTable telemetry = nt.getTable("/rosbots/TeachingPendant/Telemetry");
    private final BooleanSubscriber manualEnabled =
        manual.getBooleanTopic("Enabled").subscribe(false);
    private final BooleanSubscriber pushMode = manual.getBooleanTopic("PushMode").subscribe(false);
    private final BooleanPublisher enabledPublisher = manual.getBooleanTopic("Enabled").publish();
    private final BooleanPublisher pushModePublisher = manual.getBooleanTopic("PushMode").publish();
    private final DoublePublisher heartbeatPublisher = manual.getDoubleTopic("Heartbeat").publish();
    private final DoublePublisher translationXPublisher =
        manual.getDoubleTopic("TranslationX").publish();
    private final DoublePublisher translationYPublisher =
        manual.getDoubleTopic("TranslationY").publish();
    private final DoublePublisher rotationPublisher = manual.getDoubleTopic("Rotation").publish();
    private final BooleanSubscriber manualAccepted =
        telemetry.getBooleanTopic("ManualControlAccepted").subscribe(false);
    private final StringSubscriber driverStationMode =
        telemetry.getStringTopic("DriverStationMode").subscribe("Disabled");
    private final DoubleSubscriber robotX = telemetry.getDoubleTopic("X").subscribe(0.0);
    private final DoubleSubscriber robotY = telemetry.getDoubleTopic("Y").subscribe(0.0);
    private final DoubleSubscriber robotHeading =
        telemetry.getDoubleTopic("HeadingDegrees").subscribe(0.0);
    private final DefaultListModel<Path> autos = new DefaultListModel<>();
    private final JList<Path> autoList = new JList<>(autos);
    private final JTextField name = new JTextField();
    private final JSpinner maximumTime = new JSpinner(new SpinnerNumberModel(15.0, 0.1, 15.0, 0.1));
    private final JTextField server = new JTextField("10.55.72.2", 14);
    private final JLabel estimate = new JLabel("Estimated: —");
    private final JLabel connectionStatus = new JLabel("Offline — local auto builder ready");
    private final JLabel teachStatus = new JLabel("Connect to a robot or simulator");
    private Process simulatorProcess;
    private final DefaultMutableTreeNode commandRoot = new DefaultMutableTreeNode("Auto commands");
    private final JTree commandTree = new JTree(commandRoot);
    private JrtpAuto current;
    private Path currentFile;

    private TeachingPendantApp() {}

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new TeachingPendantApp().show());
    }

    private void show() {
        JFrame frame = new JFrame("rosbots Teaching Pendant");
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.setLayout(new BorderLayout(8, 8));
        frame.add(makeConnectionPanel(), BorderLayout.NORTH);
        frame.add(new JScrollPane(autoList), BorderLayout.WEST);
        frame.add(makeEditor(), BorderLayout.CENTER);
        frame.add(makeTeachPanel(), BorderLayout.SOUTH);
        frame.setSize(760, 480);
        frame.setLocationByPlatform(true);
        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent event) {
                stopManual();
                nt.stopClient();
                if (simulatorProcess != null && simulatorProcess.isAlive())
                    simulatorProcess.destroy();
            }
        });
        autoList.addListSelectionListener(event -> {
            if (!event.getValueIsAdjusting())
                loadSelected();
        });
        refreshAutos();
        new Timer(100, event -> {
            boolean accepted = manualAccepted.get();
            String mode = driverStationMode.get();
            connectionStatus.setText(nt.isConnected()
                ? (accepted ? "Connected — joystick accepted"
                    : "Connected — enable Test or Teleop for joystick")
                : "Offline — local auto builder ready");
            teachStatus.setText(accepted ? "Joystick is live (" + mode + ")"
                : "Joystick locked — select Test or Teleop and enable the Driver Station");
            if (manualEnabled.get()) {
                heartbeatPublisher.set(System.nanoTime() / 1_000_000_000.0);
            }
        }).start();
        frame.setVisible(true);
    }

    private JPanel makeConnectionPanel() {
        JPanel panel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        JButton connect = new JButton("Connect");
        JButton localSimulator = new JButton("Start local simulator");
        connect.addActionListener(event -> {
            nt.stopClient();
            nt.setServer(server.getText().trim());
            nt.startClient4("rosbots Teaching Pendant");
        });
        localSimulator.addActionListener(event -> startLocalSimulator());
        panel.add(new JLabel("Robot NT server:"));
        panel.add(server);
        panel.add(connect);
        panel.add(localSimulator);
        panel.add(connectionStatus);
        return panel;
    }

    private JPanel makeEditor() {
        JPanel panel = new JPanel(new BorderLayout(6, 6));
        JPanel fields = new JPanel(new GridLayout(0, 2, 6, 6));
        fields.setBorder(BorderFactory.createTitledBorder("Autonomous builder"));
        fields.add(new JLabel("Name"));
        fields.add(name);
        fields.add(new JLabel("Maximum auto time (s)"));
        fields.add(maximumTime);
        fields.add(new JLabel("Simulation"));
        fields.add(estimate);
        panel.add(fields, BorderLayout.NORTH);
        JPanel actions = new JPanel(new FlowLayout(FlowLayout.LEFT));
        JButton create = new JButton("New auto");
        JButton checkpoint = new JButton("Add checkpoint");
        JButton drive = new JButton("Add drive-to-pose");
        JButton save = new JButton("Save .jrtp");
        JButton load = new JButton("Load auto to robot");
        create.addActionListener(event -> newAuto());
        checkpoint.addActionListener(event -> addCheckpoint());
        drive.addActionListener(event -> addDrive());
        save.addActionListener(event -> save());
        load.addActionListener(event -> loadToRobot());
        actions.add(create);
        actions.add(checkpoint);
        actions.add(drive);
        actions.add(save);
        actions.add(load);
        panel.add(actions, BorderLayout.CENTER);
        panel.add(new JScrollPane(commandTree), BorderLayout.SOUTH);
        return panel;
    }

    private JPanel makeTeachPanel() {
        JPanel panel = new JPanel(new BorderLayout(8, 4));
        JSlider translationLimit = new JSlider(10, 100, 40);
        TeachDrivePanel drivePad = new TeachDrivePanel(translationLimit);
        JTextArea scopeInfo = new JTextArea("Visualization is provided by AdvantageScope.\n"
            + "Connect AdvantageScope to NT4 at 127.0.0.1:5810 for local sim, or the robot IP:5810.\n"
            + "Use the Field tab for the Field2d robot pose and 3D Field for Viz/EstState + Viz/ActualState.");
        scopeInfo.setEditable(false);
        scopeInfo.setLineWrap(true);
        scopeInfo.setWrapStyleWord(true);
        JButton stop = new JButton("STOP");
        JButton pushRobot = new JButton("Enable Push Robot (disabled only)");
        stop.addActionListener(event -> stopManual());
        pushRobot.addActionListener(event -> {
            boolean enable = !pushMode.get();
            pushModePublisher.set(enable);
            pushRobot.setText(enable ? "Disable Push Robot" : "Enable Push Robot (disabled only)");
        });
        panel.setBorder(
            BorderFactory.createTitledBorder("Teach mode (enable Test or Teleop first)"));
        JPanel limits = new JPanel(new GridLayout(0, 1));
        limits.add(new JLabel("Translation Speed"));
        limits.add(translationLimit);
        limits.add(stop);
        limits.add(pushRobot);
        JPanel center = new JPanel(new BorderLayout());
        center.add(teachStatus, BorderLayout.NORTH);
        center.add(scopeInfo, BorderLayout.CENTER);
        panel.add(drivePad, BorderLayout.WEST);
        panel.add(center, BorderLayout.CENTER);
        panel.add(limits, BorderLayout.EAST);
        return panel;
    }

    private void refreshAutos() {
        autos.clear();
        try {
            Files.createDirectories(AUTOS);
            Files.list(AUTOS).filter(path -> path.toString().endsWith(".jrtp"))
                .sorted(Comparator.comparing(Path::toString)).forEach(autos::addElement);
        } catch (Exception exception) {
            showError(exception);
        }
    }

    private void startLocalSimulator() {
        if (simulatorProcess != null && simulatorProcess.isAlive()) {
            server.setText("127.0.0.1");
            nt.stopClient();
            nt.setServer("127.0.0.1");
            nt.startClient4("rosbots Teaching Pendant");
            return;
        }
        try {
            simulatorProcess = new ProcessBuilder("./gradlew", "simulateJava")
                .directory(Path.of(System.getProperty("user.dir")).toFile())
                .redirectErrorStream(true).start();
            server.setText("127.0.0.1");
            nt.stopClient();
            nt.setServer("127.0.0.1");
            nt.startClient4("rosbots Teaching Pendant");
        } catch (Exception exception) {
            showError(new IllegalStateException(
                "Could not start ./gradlew simulateJava: " + exception.getMessage()));
        }
    }

    private void newAuto() {
        current = new JrtpAuto();
        currentFile = null;
        name.setText(current.name);
        maximumTime.setValue(current.maximumTime);
        refreshCommandTree();
        estimate();
    }

    private void loadSelected() {
        try {
            currentFile = autoList.getSelectedValue();
            if (currentFile == null)
                return;
            current = JrtpFiles.load(currentFile);
            name.setText(current.name);
            maximumTime.setValue(current.maximumTime);
            refreshCommandTree();
            estimate();
        } catch (Exception exception) {
            showError(exception);
        }
    }

    private void addCheckpoint() {
        ensureAuto();
        String checkpointName = JOptionPane.showInputDialog(null, "Checkpoint name:", "FirstScore");
        if (checkpointName == null || checkpointName.isBlank())
            return;
        JrtpAuto.Step step = new JrtpAuto.Step();
        step.type = "checkpoint";
        step.name = checkpointName;
        JrtpAuto.Input level = new JrtpAuto.Input();
        level.type = "selection";
        level.defaultValue = "L4";
        level.options.add("L1");
        level.options.add("L2");
        level.options.add("L3");
        level.options.add("L4");
        step.inputs.put("scoringLevel", level);
        current.steps.add(step);
        refreshCommandTree();
    }

    private void addDrive() {
        ensureAuto();
        try {
            JrtpAuto.Step step = new JrtpAuto.Step();
            step.type = "driveToPose";
            NetworkTable pose = nt.getTable("/rosbots/TeachingPendant/Telemetry");
            if (nt.isConnected()) {
                step.x = robotX.get();
                step.y = robotY.get();
                step.rotationDegrees = robotHeading.get();
            } else {
                step.x = Double.parseDouble(JOptionPane.showInputDialog(null, "X meters:", "3.0"));
                step.y = Double.parseDouble(JOptionPane.showInputDialog(null, "Y meters:", "5.0"));
                step.rotationDegrees = Double
                    .parseDouble(JOptionPane.showInputDialog(null, "Heading degrees:", "180"));
            }
            step.maxSpeed = 2.5;
            current.steps.add(step);
            refreshCommandTree();
            estimate();
        } catch (NumberFormatException ignored) {
        }
    }

    private void save() {
        try {
            ensureAuto();
            current.name = name.getText().trim();
            current.maximumTime = (double) maximumTime.getValue();
            if (current.name.isBlank())
                throw new IllegalArgumentException("An auto needs a name");
            if (currentFile == null)
                currentFile =
                    AUTOS.resolve(current.name.replaceAll("[^A-Za-z0-9_-]", "_") + ".jrtp");
            JrtpFiles.save(currentFile, current);
            refreshAutos();
            estimate();
        } catch (Exception exception) {
            showError(exception);
        }
    }

    private void loadToRobot() {
        save();
        if (!nt.isConnected()) {
            JOptionPane.showMessageDialog(null,
                "The auto was saved locally. Connect to the robot later to load it.",
                "Auto saved offline", JOptionPane.INFORMATION_MESSAGE);
            return;
        }
        if (current != null)
            try {
                NetworkTable autonomous = nt.getTable(AudibleManager.ROOT);
                autonomous.getEntry("RequestedAuto")
                    .setString(currentFile.getFileName().toString());
                autonomous.getEntry("RequestedAutoJson").setString(JrtpFiles.encode(current));
            } catch (Exception exception) {
                showError(exception);
            }
    }

    private void estimate() {
        if (current == null) {
            estimate.setText("Estimated: —");
            return;
        }
        double x = current.startingPose.x, y = current.startingPose.y, seconds = 0.0;
        for (JrtpAuto.Step step : current.steps)
            if ("driveToPose".equals(step.type) && step.x != null && step.y != null) {
                seconds += Math.hypot(step.x - x, step.y - y)
                    / Math.max(.1, step.maxSpeed == null ? 2.5 : step.maxSpeed);
                x = step.x;
                y = step.y;
            }
        estimate.setText(String.format("Estimated: %.1f s / %.1f s", seconds, current.maximumTime));
    }

    private void ensureAuto() {
        if (current == null)
            newAuto();
    }

    private void refreshCommandTree() {
        commandRoot.removeAllChildren();
        commandRoot.setUserObject(current == null ? "Auto commands" : current.name);
        if (current != null)
            for (JrtpAuto.Step step : current.steps) {
                String label = switch (step.type) {
                    case "driveToPose" -> String.format("Drive to (%.2f, %.2f, %.0f°)", step.x,
                        step.y, step.rotationDegrees);
                    case "checkpoint" -> "Checkpoint: " + step.name + "  " + step.inputs.keySet();
                    case "existingCommand" -> "Command: " + step.command;
                    default -> step.type;
                };
                commandRoot.add(new DefaultMutableTreeNode(label));
            }
        ((DefaultTreeModel) commandTree.getModel()).reload();
        for (int row = 0; row < commandTree.getRowCount(); row++)
            commandTree.expandRow(row);
    }

    private void stopManual() {
        enabledPublisher.set(false);
        translationXPublisher.set(0.0);
        translationYPublisher.set(0.0);
        rotationPublisher.set(0.0);
    }

    /** Drag the center puck to command translation. */
    private final class TeachDrivePanel extends JPanel {
        private final JSlider translationLimit;
        private double x;
        private double y;

        TeachDrivePanel(JSlider translationLimit) {
            this.translationLimit = translationLimit;
            setPreferredSize(new Dimension(190, 150));
            addMouseListener(new java.awt.event.MouseAdapter() {
                @Override
                public void mousePressed(java.awt.event.MouseEvent event) {
                    command(event);
                }

                @Override
                public void mouseReleased(java.awt.event.MouseEvent event) {
                    stopManual();
                    x = y = 0;
                    repaint();
                }
            });
            addMouseMotionListener(new java.awt.event.MouseMotionAdapter() {
                @Override
                public void mouseDragged(java.awt.event.MouseEvent event) {
                    command(event);
                }
            });
        }

        private void command(java.awt.event.MouseEvent event) {
            int cx = getWidth() / 2, cy = getHeight() / 2;
            x = Math.max(-1, Math.min(1, (event.getX() - cx) / (double) (cx - 16)));
            y = Math.max(-1, Math.min(1, (cy - event.getY()) / (double) (cy - 16)));
            double translate = translationLimit.getValue() / 100.0;
            translationXPublisher.set(x * translate);
            translationYPublisher.set(y * translate);
            rotationPublisher.set(0.0);
            enabledPublisher.set(true);
            repaint();
        }

        @Override
        protected void paintComponent(Graphics graphics) {
            super.paintComponent(graphics);
            Graphics2D g = (Graphics2D) graphics;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            int cx = getWidth() / 2, cy = getHeight() / 2;
            g.setColor(new Color(235, 235, 235));
            g.fillRoundRect(8, 8, getWidth() - 16, getHeight() - 16, 12, 12);
            g.setColor(Color.GRAY);
            g.drawLine(cx, 14, cx, getHeight() - 14);
            g.drawLine(14, cy, getWidth() - 14, cy);
            g.setColor(new Color(0, 100, 190));
            g.fillOval((int) (cx + x * (cx - 22)) - 12, (int) (cy - y * (cy - 22)) - 12, 24, 24);
            g.setColor(Color.DARK_GRAY);
            g.drawString("Drag to translate", 15, getHeight() - 7);
        }
    }

    /** Simple live field graphic backed by pose telemetry published by the robot. */
    private final class RobotFieldPanel extends JPanel {
        RobotFieldPanel() {
            setPreferredSize(new Dimension(260, 150));
            new Timer(50, event -> repaint()).start();
        }

        @Override
        protected void paintComponent(Graphics graphics) {
            super.paintComponent(graphics);
            Graphics2D g = (Graphics2D) graphics;
            g.setColor(new Color(42, 106, 55));
            g.fillRect(0, 0, getWidth(), getHeight());
            NetworkTable telemetry = nt.getTable("/rosbots/TeachingPendant/Telemetry");
            double x = robotX.get();
            double y = robotY.get();
            double heading = robotHeading.get();
            int px = (int) (12 + Math.max(0, Math.min(1, x / 16.54)) * (getWidth() - 24));
            int py =
                (int) (getHeight() - 12 - Math.max(0, Math.min(1, y / 8.21)) * (getHeight() - 24));
            g.setColor(Color.WHITE);
            g.drawRect(10, 10, getWidth() - 20, getHeight() - 20);
            g.translate(px, py);
            g.rotate(-Math.toRadians(heading));
            g.setColor(new Color(255, 190, 0));
            g.fillRect(-12, -9, 24, 18);
            g.setColor(Color.BLACK);
            g.fillPolygon(new int[] {12, 20, 12}, new int[] {-7, 0, 7}, 3);
            g.rotate(Math.toRadians(heading));
            g.translate(-px, -py);
            g.setColor(Color.WHITE);
            g.drawString(String.format("X %.2f  Y %.2f  %.0f°", x, y, heading), 14, 22);
        }
    }

    private static void showError(Exception exception) {
        JOptionPane.showMessageDialog(null, exception.getMessage(), "Teaching Pendant",
            JOptionPane.ERROR_MESSAGE);
    }
}
