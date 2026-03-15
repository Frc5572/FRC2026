package org.frc5572;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.TimeZone;
import java.util.concurrent.TimeUnit;
import org.gradle.api.DefaultTask;
import org.gradle.api.tasks.TaskAction;

/**
 * Task to modify BuildConstants to give its static fields their proper values. This is done
 * in-place, after compilation is completed (therefore not triggering a recompile).
 */
public class BuildConstantsGen extends DefaultTask {

    private byte[] classBuffer = new byte[0];

    private String executeGetOutput(String[] cmd, String orElse) {
        ProcessBuilder pb = new ProcessBuilder(cmd);
        try {
            Process p = pb.start();
            p.waitFor(5000, TimeUnit.MILLISECONDS);
            String res = new String(p.getInputStream().readAllBytes());
            if (res.isBlank()) {
                System.err.println("command returned an empty string");
                System.err.println("pwd     = " + new File(".").getAbsolutePath());
                System.err.print("command = [");
                for (int i = 0; i < cmd.length; i++) {
                    if (i != 0) {
                        System.err.print(", ");
                    }
                    System.err.print(cmd[i]);
                }
                System.err.println("]");
                return orElse;
            }
            if (p.exitValue() != 0) {
                System.err.println("command returned a non-zero value: " + p.exitValue());
                System.err.println("pwd     = " + new File(".").getAbsolutePath());
                System.err.print("command = [");
                for (int i = 0; i < cmd.length; i++) {
                    if (i != 0) {
                        System.err.print(", ");
                    }
                    System.err.print(cmd[i]);
                }
                System.err.println("]");
                return orElse;
            }
            return res;
        } catch (InterruptedException | IOException e) {
            e.printStackTrace();
            return orElse;
        }
    }

    private int executeGetSuccess(String[] cmd) {
        ProcessBuilder pb = new ProcessBuilder(cmd);
        try {
            Process p = pb.start();
            p.waitFor(5000, TimeUnit.MILLISECONDS);
            return p.exitValue();
        } catch (InterruptedException | IOException e) {
            e.printStackTrace();
            return -1;
        }
    }

    private static int[] parseGitVersion(String text) {
        int[] version = new int[3];

        if (text == null) {
            return version;
        }
        String[] words = text.split("\\s+");
        if (words.length != 3) {
            return version;
        }

        words = words[2].split("\\.");
        if (words.length < 1) {
            return version;
        }

        for (int i = 0; i < Math.min(version.length, words.length); i++) {
            version[i] = Integer.parseInt(words[i]);
        }

        return version;
    }

    /** Main task for this plugin. */
    @TaskAction
    public void run() throws IOException, ParseException {
        var file = getProject().getLayout().getBuildDirectory()
            .file("classes/java/main/frc/robot/BuildConstants.class").get();
        classBuffer = Files.readAllBytes(file.getAsFile().toPath());

        int[] version_of_git =
            parseGitVersion(executeGetOutput(new String[] {"git", "version"}, "UNKNOWN"));

        int dirty_value =
            executeGetSuccess(new String[] {"git", "diff", "--quiet", "--ignore-submodules=dirty"});
        int git_revision = Integer.parseInt(executeGetOutput(
            new String[] {"git", "rev-list", "--no-show-signature", "--count", "HEAD"}, "-1")
                .strip());

        String git_date;
        String date_format;

        if (version_of_git[0] == 1) {
            date_format = "yyyy-MM-dd HH:mm:ss Z";
            git_date = executeGetOutput(
                new String[] {"git", "show", "--no-show-signature", "-s", "--format=%ci", "HEAD"},
                "UNKNOWN");
        } else {
            date_format = "yyyy-MM-dd'T'HH:mm:ssXXX";
            git_date = executeGetOutput(
                new String[] {"git", "show", "--no-show-signature", "-s", "--format=%cI", "HEAD"},
                "UNKNOWN");
        }

        TimeZone tz = TimeZone.getTimeZone("UTC");

        if (!git_date.equals("UNKNOWN")) {
            try {
                // Let's convert this from local to the desired time zone and adjust the date format
                DateFormat formatter = new SimpleDateFormat(date_format);
                Date java_data = formatter.parse(git_date);
                formatter = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'");
                formatter.setTimeZone(tz);
                git_date = formatter.format(java_data);
            } catch (RuntimeException e) {
                e.printStackTrace(System.err);
                git_date = "UNKNOWN";
            }
        }

        var build_unix_time = System.currentTimeMillis();
        DateFormat formatter = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'");
        formatter.setTimeZone(tz);
        String build_date = formatter.format(new Date(build_unix_time));

        Map<String, String> strings = new HashMap<>();

        strings.put("MAVEN_GROUP__", getProject().getGroup().toString());
        strings.put("MAVEN_NAME__", getProject().getName());
        strings.put("VERSION__", getProject().getVersion().toString());
        strings.put("GIT_SHA__",
            executeGetOutput(new String[] {"git", "rev-parse", "HEAD"}, "UNKNOWN"));
        strings.put("GIT_DATE__", git_date);
        strings.put("GIT_BRANCH__",
            executeGetOutput(new String[] {"git", "rev-parse", "--abbrev-ref", "HEAD"}, "UNKNOWN"));
        strings.put("BUILD_DATE__", build_date);

        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();

        int constantPoolCount = readUnsignedShort(8);
        int currentCpInfoIndex = 1;
        int currentCpInfoOffset = 10;

        outputStream.write(classBuffer, 0, currentCpInfoOffset);

        while (currentCpInfoIndex < constantPoolCount) {
            currentCpInfoIndex++;
            int cpInfoSize;
            boolean isReplaced = false;
            switch (classBuffer[currentCpInfoOffset]) {
                case 1:
                    int size = readUnsignedShort(currentCpInfoOffset + 1);
                    cpInfoSize = 3 + size; {
                    String value = readUtf(currentCpInfoOffset + 3, size);
                    if (strings.containsKey(value)) {
                        outputStream.write((byte) 1);
                        putString(outputStream, strings.get(value));
                        isReplaced = true;
                    }
                }
                    break;
                case 3:
                    cpInfoSize = 5; {
                    int value = readInt(currentCpInfoOffset + 1);
                    if (value == 65536) {
                        outputStream.write((byte) 3);
                        putInt(outputStream, git_revision);
                        isReplaced = true;
                    } else if (value == 65588) {
                        outputStream.write((byte) 3);
                        putInt(outputStream, dirty_value);
                        isReplaced = true;
                    }
                }
                    break;
                case 5:
                    cpInfoSize = 9; {
                    long value = readLong(currentCpInfoOffset + 1);
                    if (value == 65567L) {
                        outputStream.write((byte) 5);
                        putLong(outputStream, build_unix_time);
                        isReplaced = true;
                    }
                    currentCpInfoIndex++;
                }
                    break;
                case 7:
                case 8:
                case 16:
                case 19:
                case 20:
                    cpInfoSize = 3;
                    break;
                case 15:
                    cpInfoSize = 4;
                    break;
                case 4:
                case 9:
                case 10:
                case 11:
                case 12:
                case 17:
                case 18:
                    cpInfoSize = 5;
                    break;
                case 6:
                    cpInfoSize = 9;
                    currentCpInfoIndex++;
                    break;
                default:
                    throw new IllegalArgumentException(
                        "Invalid ConstantPool Type: " + classBuffer[currentCpInfoOffset]);
            }
            if (!isReplaced) {
                outputStream.write(classBuffer, currentCpInfoOffset, cpInfoSize);
            }
            currentCpInfoOffset += cpInfoSize;
        }

        outputStream.write(classBuffer, currentCpInfoOffset,
            classBuffer.length - currentCpInfoOffset);

        Files.write(file.getAsFile().toPath(), outputStream.toByteArray());
    }

    private int readUnsignedShort(int offset) {
        return ((classBuffer[offset] & 0xFF) << 8) | (classBuffer[offset + 1] & 0xFF);
    }

    private int readInt(final int offset) {
        return ((classBuffer[offset] & 0xFF) << 24) | ((classBuffer[offset + 1] & 0xFF) << 16)
            | ((classBuffer[offset + 2] & 0xFF) << 8) | (classBuffer[offset + 3] & 0xFF);
    }

    private long readLong(final int offset) {
        long l1 = readInt(offset);
        long l0 = readInt(offset + 4) & 0xFFFFFFFFL;
        return (l1 << 32) | l0;
    }

    private String readUtf(final int utfOffset, final int utfLength) {
        char[] charBuffer = new char[utfLength];
        int currentOffset = utfOffset;
        int endOffset = currentOffset + utfLength;
        int strLength = 0;
        while (currentOffset < endOffset) {
            int currentByte = classBuffer[currentOffset++];
            if ((currentByte & 0x80) == 0) {
                charBuffer[strLength++] = (char) (currentByte & 0x7F);
            } else if ((currentByte & 0xE0) == 0xC0) {
                charBuffer[strLength++] =
                    (char) (((currentByte & 0x1F) << 6) + (classBuffer[currentOffset++] & 0x3F));
            } else {
                charBuffer[strLength++] = (char) (((currentByte & 0xF) << 12)
                    + ((classBuffer[currentOffset++] & 0x3F) << 6)
                    + (classBuffer[currentOffset++] & 0x3F));
            }
        }
        return new String(charBuffer, 0, strLength);
    }

    private void putInt(ByteArrayOutputStream baos, int intValue) {
        byte[] res = new byte[4];
        res[0] = (byte) (intValue >>> 24);
        res[1] = (byte) (intValue >>> 16);
        res[2] = (byte) (intValue >>> 8);
        res[3] = (byte) (intValue >>> 0);
        baos.write(res, 0, 4);
    }

    private void putLong(ByteArrayOutputStream baos, long longValue) {
        int intValue = (int) (longValue >>> 32);
        putInt(baos, intValue);
        intValue = (int) longValue;
        putInt(baos, intValue);
    }

    private void putString(ByteArrayOutputStream baos, String strValue) {
        byte[] res = new byte[2 + strValue.length()];
        int len = strValue.length();
        res[0] = (byte) (len >>> 8);
        res[1] = (byte) len;
        for (int i = 0; i < len; i++) {
            res[2 + i] = (byte) strValue.charAt(i);
        }
        baos.write(res, 0, len + 2);
    }

}
