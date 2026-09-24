package frc.robot.config;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Set;
import java.util.TreeSet;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The robot's writable configuration directory, shared by every live-tunable subsystem.
 *
 * <h2>Why two locations</h2> The deploy directory is read-only to {@code lvuser} on the roboRIO,
 * so live edits cannot be written back to it. The authoritative copy lives on the USB stick
 * alongside the match logs, and the copy under {@code src/main/deploy} acts as a seed: it is what
 * a fresh robot boots with, and it is what {@code ./gradlew pullConfigs} writes back into git.
 * Precedence on load is USB, then the deployed seed, then nothing.
 *
 * <h2>Why a singleton</h2> The directory is served over HTTP so the build can retrieve it without
 * SSH, and a port can only be bound once. Several subsystems keep configuration here, so they
 * share one store rather than each starting a server.
 *
 * <h2>The manifest</h2> {@link WebServer} serves known paths and offers no directory index, so a
 * build task cannot discover what is here by asking. Each consumer {@link #register}s its
 * document at startup and the store publishes {@value #MANIFEST} listing them; the pull task
 * fetches that first and then each file it names.
 */
@NullMarked
public final class ConfigStore {

    /** Port serving the configuration directory, so the build can pull it over HTTP. */
    public static final int WEB_PORT = 5801;

    /** Index of the documents in this directory, for {@code ./gradlew pullConfigs}. */
    public static final String MANIFEST = "manifest.json";

    /** Subdirectory used under whichever writable root is available. */
    public static final String DIR_NAME = "controls";

    /** Candidate writable roots on a real robot, most preferred first. */
    private static final String[] ROBOT_ROOTS = {"/media/sda1", "/media/sdb1", "/home/lvuser"};

    private static @Nullable ConfigStore instance;

    private final Path directory;
    private final Set<String> registered = new TreeSet<>();
    private @Nullable String lastError;

    private ConfigStore() {
        this.directory = resolveDirectory();
        startWebServer();
    }

    /** The shared store, creating and starting it on first use. */
    public static synchronized ConfigStore getInstance() {
        if (instance == null) {
            instance = new ConfigStore();
        }
        return instance;
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

    private void startWebServer() {
        try {
            Files.createDirectories(directory);
            WebServer.start(WEB_PORT, directory.toString());
            System.out.println("[Config] serving " + directory + " on port " + WEB_PORT);
        } catch (IOException e) {
            System.err.println("[Config] could not start config web server: " + e);
        }
    }

    /** The directory live edits are written to. Also the directory served over HTTP. */
    public Path directory() {
        return directory;
    }

    /** The path a named document is written to. */
    public Path file(String name) {
        return directory.resolve(name);
    }

    /** The read-only seed for a named document, shipped in the deploy directory. */
    public static Path seedFile(String name) {
        return Filesystem.getDeployDirectory().toPath().resolve(DIR_NAME).resolve(name);
    }

    /** The most recent read or write failure, or null if the last operation succeeded. */
    public @Nullable String lastError() {
        return lastError;
    }

    /**
     * Declare that a document belongs to this store, so the pull task can find it even before it
     * has ever been saved.
     *
     * @param name the document's file name
     */
    public void register(String name) {
        if (registered.add(name)) {
            writeManifest();
        }
    }

    /**
     * Read a document, preferring the writable copy over the deployed seed.
     *
     * @param name the document's file name
     * @return its contents, or null when neither copy is readable
     */
    public @Nullable String load(String name) {
        lastError = null;
        register(name);
        Path live = file(name);
        if (Files.isReadable(live)) {
            String json = readOrNull(live);
            if (json != null) {
                return json;
            }
        }
        Path seed = seedFile(name);
        if (Files.isReadable(seed)) {
            String json = readOrNull(seed);
            if (json != null) {
                System.out.println("[Config] seeded " + name + " from " + seed);
                return json;
            }
        }
        System.out.println("[Config] no " + name + " found, using defaults");
        return null;
    }

    private @Nullable String readOrNull(Path path) {
        try {
            return Files.readString(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            lastError = "read " + path + ": " + e.getMessage();
            System.err.println("[Config] " + lastError);
            return null;
        }
    }

    /**
     * Write a document to the writable directory.
     *
     * <p>
     * Written to a temporary file and moved into place, so a power cut part way through cannot
     * leave a truncated file behind.
     *
     * @param name the document's file name
     * @param contents the text to write
     * @return true when the file was written successfully
     */
    public boolean save(String name, String contents) {
        lastError = null;
        register(name);
        Path target = file(name);
        try {
            Files.createDirectories(directory);
            Path tmp = directory.resolve(name + ".tmp");
            Files.writeString(tmp, contents, StandardCharsets.UTF_8);
            Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING);
            writeManifest();
            System.out.println("[Config] saved " + target);
            return true;
        } catch (IOException e) {
            lastError = "write " + target + ": " + e.getMessage();
            System.err.println("[Config] " + lastError);
            return false;
        }
    }

    /**
     * Publish the index of documents actually present in this directory.
     *
     * <p>
     * Only existing files are listed. A document that is registered but has never been saved has
     * nothing to pull, and listing it would make the build ask for a file the robot would 404 on.
     */
    private void writeManifest() {
        StringBuilder sb = new StringBuilder("{\n  \"files\" : [");
        boolean first = true;
        for (String name : registered) {
            if (!Files.isReadable(directory.resolve(name))) {
                continue;
            }
            sb.append(first ? "\n    \"" : ",\n    \"").append(name).append('"');
            first = false;
        }
        sb.append(first ? "]\n}\n" : "\n  ]\n}\n");
        try {
            Files.createDirectories(directory);
            Files.writeString(directory.resolve(MANIFEST), sb.toString(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            System.err.println("[Config] could not write manifest: " + e);
        }
    }

}
