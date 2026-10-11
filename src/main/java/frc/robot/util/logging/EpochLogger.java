package frc.robot.util.logging;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Timer;
import org.wpilib.system.Tracer;

public class EpochLogger {
    private final Tracer tracer;
    private BufferedWriter writer;

    private final Timer flushTimer = new Timer();
    private static final double FLUSH_INTERVAL_SECONDS = 1.0;

    public EpochLogger(Tracer tracer) {
        this.tracer = tracer;
    }

    public void start() {
        try {
            String timestamp = LocalDateTime.now().format(
                DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")
            );

            Path logPath = Path.of("/U/epoch-logs/epochs_" + timestamp + ".txt");

            writer = Files.newBufferedWriter(logPath, StandardCharsets.UTF_8);

            flushTimer.restart();


        } catch (IOException e) {
            DriverStationErrors.reportError("Failed to create epoch log: " + e.getMessage(), e.getStackTrace());
        }
    }

    public void update() {
        if (writer == null) {
            return;
        }

        tracer.printEpochs(line -> {
            try {
                if (line.startsWith("\tSmart ")) {
                    String timestamp = LocalDateTime.now().format(
                        DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS")
                    );

                    writer.write("=== " + timestamp + "===");
                    writer.newLine();
                }
                writer.write(line);
                writer.newLine();
            } catch (IOException e) {
                DriverStationErrors.reportError("Failed to write epoch log: " + e.getMessage(), e.getStackTrace());
            }
        });

        if (flushTimer.advanceIfElapsed(FLUSH_INTERVAL_SECONDS)) {
            try {
                writer.flush();
            } catch (IOException e) {
                DriverStationErrors.reportError("Failed to flush epoch log: " + e.getMessage(), e.getStackTrace());
            }
        }
    }
    
    public void stop() {
        if (writer == null) {
            return;
        }

        try {
            writer.flush();
            writer.close();
        } catch (IOException e) {
            DriverStationErrors.reportError("Failed to close epoch log: " + e.getMessage(), e.getStackTrace());
        } finally {
            writer = null;
            flushTimer.stop();
        }
    }
}
