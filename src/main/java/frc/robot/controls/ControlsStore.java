package frc.robot.controls;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Locates and persists {@code profiles.json} on the robot.
 *
 * <h2>Why two locations</h2> The deploy directory is read-only to {@code lvuser} on the roboRIO,
 * so live edits cannot be written back to it. Instead the authoritative copy lives on the USB
 * stick alongside the match logs, and the copy under {@code src/main/deploy} acts as a seed: it is
 * what a fresh robot boots with, and it is what {@code ./gradlew pullControls} writes back into
 * git before a deploy.
 *
 * <p>
 * Precedence at boot is USB, then the deployed seed, then factory defaults. The directory chosen
 * for writing is also served over HTTP so the build can retrieve it without needing SSH.
 */
@NullMarked
public final class ControlsStore {

    /** Name of the profiles document, in both the writable directory and the deploy seed. */
    public static final String FILE_NAME = "profiles.json";

    /** Subdirectory used under whichever writable root is available. */
    public static final String DIR_NAME = "controls";

    /** Candidate writable roots on a real robot, most preferred first. */
    private static final String[] ROBOT_ROOTS = {"/media/sda1", "/media/sdb1", "/home/lvuser"};

    private final Path directory;
    private @Nullable String lastError;

    /**
     * Resolve the directory that live edits are written to.
     *
     * <p>
     * On a real robot this is {@code <usb>/controls}, falling back to {@code /home/lvuser/controls}
     * when no stick is mounted. In simulation it is the project's own deploy directory, so that
     * tuning in the sim edits the file that is already checked in.
     */
    public ControlsStore() {
        this.directory = resolveDirectory();
    }

    private static Path resolveDirectory() {
        if (!RobotBase.isReal()) {
            return Filesystem.getDeployDirectory().toPath().resolve(DIR_NAME);
        }
        for (String root : ROBOT_ROOTS) {
            File rootFile = new File(root);
            if (rootFile.isDirectory() && rootFile.canWrite()) {
                return rootFile.toPath().resolve(DIR_NAME);
            }
        }
        return new File("/home/lvuser").toPath().resolve(DIR_NAME);
    }

    /** The directory live edits are written to. Also the directory served over HTTP. */
    public Path directory() {
        return directory;
    }

    /** The file live edits are written to. */
    public Path file() {
        return directory.resolve(FILE_NAME);
    }

    /** The read-only seed shipped with the deploy directory. */
    public static Path seedFile() {
        return Filesystem.getDeployDirectory().toPath().resolve(DIR_NAME).resolve(FILE_NAME);
    }

    /** The most recent read or write failure, or null if the last operation succeeded. */
    public @Nullable String lastError() {
        return lastError;
    }

    /**
     * Load the profile set, preferring the writable copy over the deployed seed.
     *
     * @return the loaded profiles, or factory defaults when neither file is readable
     */
    public ControlsProfiles load() {
        lastError = null;
        Path live = file();
        if (Files.isReadable(live)) {
            String json = readOrNull(live);
            if (json != null) {
                return ControlsProfiles.fromJson(json);
            }
        }
        Path seed = seedFile();
        if (Files.isReadable(seed)) {
            String json = readOrNull(seed);
            if (json != null) {
                System.out.println("[Controls] seeded from " + seed);
                return ControlsProfiles.fromJson(json);
            }
        }
        System.out.println("[Controls] no profiles found, using defaults");
        return new ControlsProfiles();
    }

    private @Nullable String readOrNull(Path path) {
        try {
            return Files.readString(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            lastError = "read " + path + ": " + e.getMessage();
            System.err.println("[Controls] " + lastError);
            return null;
        }
    }

    /**
     * Write the profile set to the writable directory.
     *
     * <p>
     * The document is written to a temporary file and then moved into place, so a power cut part
     * way through a save cannot leave a truncated file behind.
     *
     * @param profiles the profiles to persist
     * @return true when the file was written successfully
     */
    public boolean save(ControlsProfiles profiles) {
        lastError = null;
        Path target = file();
        try {
            Files.createDirectories(directory);
            Path tmp = directory.resolve(FILE_NAME + ".tmp");
            Files.writeString(tmp, profiles.toJson(), StandardCharsets.UTF_8);
            Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING);
            System.out.println("[Controls] saved " + target);
            return true;
        } catch (IOException e) {
            lastError = "write " + target + ": " + e.getMessage();
            System.err.println("[Controls] " + lastError);
            return false;
        }
    }

}
