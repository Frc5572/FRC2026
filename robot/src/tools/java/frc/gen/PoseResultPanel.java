package frc.gen;

import java.awt.BorderLayout;
import java.awt.Font;
import java.util.Arrays;
import javax.swing.BorderFactory;
import javax.swing.JPanel;
import javax.swing.JTextArea;

public class PoseResultPanel extends JPanel {

    private final JTextArea textArea;

    public PoseResultPanel() {
        setLayout(new BorderLayout());
        setBorder(BorderFactory.createTitledBorder("Camera Pose"));

        textArea = new JTextArea(8, 28);
        textArea.setEditable(false);
        textArea.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        textArea.setText("No tags detected.");
        add(textArea, BorderLayout.CENTER);
    }

    public void update(PoseResult result) {
        if (result == null) {
            textArea.setText("No tags detected.");
            return;
        }

        double[] t = result.translation();
        double[] r = result.rvec();
        String text = String.format(
            "Tags: %s%n%nPosition (m):%n  x: %6.3f%n  y: %6.3f%n  z: %6.3f%n%nRotation (rad):%n  rx: %6.3f%n  ry: %6.3f%n  rz: %6.3f",
            Arrays.toString(result.detectedTagIds()),
            t[0], t[1], t[2],
            r[0], r[1], r[2]
        );
        textArea.setText(text);
    }
}
