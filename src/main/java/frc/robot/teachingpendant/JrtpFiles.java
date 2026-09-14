package frc.robot.teachingpendant;

import java.io.IOException;
import java.nio.file.Path;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

/** Validated reader/writer for .jrtp files. */
public final class JrtpFiles {
    private static final ObjectMapper mapper = new ObjectMapper()
        .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
        .enable(SerializationFeature.INDENT_OUTPUT);

    private JrtpFiles() {}

    public static JrtpAuto load(Path file) throws IOException {
        JrtpAuto auto = mapper.readValue(file.toFile(), JrtpAuto.class);
        validate(auto);
        return auto;
    }

    public static JrtpAuto decode(String json) throws IOException {
        JrtpAuto auto = mapper.readValue(json, JrtpAuto.class);
        validate(auto);
        return auto;
    }

    public static String encode(JrtpAuto auto) throws IOException {
        validate(auto);
        return mapper.writeValueAsString(auto);
    }

    private static void validate(JrtpAuto auto) throws IOException {
        if (!"jrtp".equals(auto.format) || auto.version != 1) {
            throw new IOException("Unsupported .jrtp format or version");
        }
        if (auto.name == null || auto.name.isBlank() || auto.maximumTime <= 0.0) {
            throw new IOException("Auto needs a name and a positive maximumTime");
        }
    }

    public static void save(Path file, JrtpAuto auto) throws IOException {
        if (!file.getFileName().toString().endsWith(".jrtp")) {
            throw new IOException("Teaching-pendant autos must use the .jrtp extension");
        }
        auto.format = "jrtp";
        auto.version = 1;
        validate(auto);
        mapper.writeValue(file.toFile(), auto);
    }
}
