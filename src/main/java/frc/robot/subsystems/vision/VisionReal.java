package frc.robot.subsystems.vision;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import org.apache.http.HttpEntity;
import org.apache.http.client.methods.CloseableHttpResponse;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.entity.mime.content.FileBody;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** PhotonVision-attached implementation */
@NullMarked
public class VisionReal implements VisionIO {

    protected final PhotonCamera[] cameras;

    /** PhotonVision-attached implementation */
    public VisionReal() {
        cameras = Arrays.stream(Constants.Vision.cameraConstants)
            .map((consts) -> new PhotonCamera(consts.name)).toArray(PhotonCamera[]::new);
        String[] coprocessorNames = Arrays.stream(Constants.Vision.cameraConstants)
            .map((consts) -> consts.coProcessorName).toArray(String[]::new);
        for (String hostname : coprocessorNames) {
            System.out.println("PLACEHOLDER --- Uploading settings to Photon Vision: " + hostname);
            createSettingsUploadThread(hostname);
        }
    }

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        for (int i = 0; i < cameras.length; i++) {
            inputs[i].results =
                cameras[i].getAllUnreadResults().toArray(PhotonPipelineResult[]::new);
            inputs[i].cameraMatrix = cameras[i].getCameraMatrix();
            inputs[i].distCoeffs = cameras[i].getDistCoeffs();
        }
    }

    /**
     * Upload saved settings to PV Co-Processors
     *
     * @param hostname Hostname of Co-Processor
     * @return True when completed
     * @throws IOException IOException for HTTP request
     */
    public boolean uploadSettings(String hostname) throws IOException {
        if (hostname == "skip") {
            return true;
        }
        System.out.println("Uploading ALL settings for PV: " + hostname);
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpPost postReq = new HttpPost("http://" + hostname + ".local:5800/api/settings");
            HttpEntity entity = MultipartEntityBuilder.create()
                .addPart("data", new FileBody(new File(Filesystem.getDeployDirectory().getPath()
                    + "/localization/" + hostname + ".zip")))
                .build();
            postReq.setEntity(entity);
            try (CloseableHttpResponse response = httpClient.execute(postReq)) {
                Logger.recordOutput("uploadSettings/" + hostname + "/AllSettings/status",
                    response.getStatusLine().getStatusCode() + ": "
                        + response.getStatusLine().getReasonPhrase());
                var ent = response.getEntity();
                if (ent != null) {
                    try (InputStream stream = ent.getContent()) {
                        String text = new String(stream.readAllBytes(), StandardCharsets.UTF_8);
                        Logger.recordOutput("uploadSettings/" + hostname + "/AllSettings/content",
                            text);
                    }
                } else {
                    Logger.recordOutput("uploadSettings/" + hostname + "/AllSettings/content",
                        "null");
                }
                return response.getStatusLine().getStatusCode() == 200;
            }
        }
    }

    /**
     * Check that PV has started on Co-Processor
     *
     * @param hostname Hostname of Co-Processor
     * @return True when completed
     * @throws IOException IOException for HTTP request
     */
    public boolean waitForPV(String hostname) throws IOException {
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpGet getReq = new HttpGet("http://" + hostname + ":5800");
            try (CloseableHttpResponse response = httpClient.execute(getReq)) {
                Logger.recordOutput("uploadSettings/" + hostname + "/status",
                    response.getStatusLine().getStatusCode() + ": "
                        + response.getStatusLine().getReasonPhrase());
                if (response.getStatusLine().getStatusCode() == 200) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Create a thread to upload PV settings/April Tag field
     *
     * @param hostname hostname of the PV Co-Processor
     */
    protected void createSettingsUploadThread(String hostname) {
        System.out.println("Starting upload thread for PV: " + hostname);
        new Thread(() -> {
            Timer timer = new Timer();
            boolean run = true;
            int counter = 0;
            timer.start();
            while (run) {
                if (counter >= 12) {
                    run = false;
                }
                if (timer.advanceIfElapsed(5.0)) {
                    try {
                        if (!waitForPV(hostname)) {
                            continue;
                        }
                        if (uploadSettings(hostname)) {
                            run = false;
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    counter++;
                }
                Thread.yield();
            }
        }).start();
    }

}
