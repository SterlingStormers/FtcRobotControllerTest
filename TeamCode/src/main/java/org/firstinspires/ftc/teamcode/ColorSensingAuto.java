package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.Optional;

public class ColorSensingAuto {

    private PredominantColorProcessor colorSensor;
    private VisionPortal portal;
    private FtcDashboard dashboard;

    // State variables
    public volatile boolean colorReady = false;         // True when color scan is complete
    public volatile PredominantColorProcessor.Swatch detectedColor = null;
    public volatile boolean scanning = false;          // True while scan is in progress
    private long scanStartTime = 0;                     // For timing the 50ms scan

    public ColorSensingAuto(OpMode opMode, String webcamName) {
        WebcamName webcam = opMode.hardwareMap.get(WebcamName.class, webcamName);

        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.2, 0.2, 0.2, -0.2))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.CYAN
                )
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(webcam)
                .build();

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(portal, 30);
    }

    // Call this once to start a 50ms scan
    public void startScan() {
        scanning = true;
        colorReady = false;
        scanStartTime = System.currentTimeMillis();
    }

    // Call this repeatedly in your auto loop/state machine
    public void update() {
        if (scanning) {
            long elapsed = System.currentTimeMillis() - scanStartTime;

            // Send heartbeat telemetry to dashboard so you can see the scan is running
            TelemetryPacket heartbeat = new TelemetryPacket();
            heartbeat.put("scanning", true);
            heartbeat.put("elapsed_ms", elapsed);
            dashboard.sendTelemetryPacket(heartbeat);

            // Use 100ms for debug (change to 50 or 200 once stable)
            if (elapsed >= 100) {
                PredominantColorProcessor.Result result = colorSensor.getAnalysis();

                if (result != null) {
                    detectedColor = result.closestSwatch; // may be null too
                } else {
                    detectedColor = null;
                }

                colorReady = true;   // scan is done
                scanning = false;

                // Detailed result telemetry
                TelemetryPacket resultPacket = new TelemetryPacket();
                resultPacket.put("Best Match (object)", (result == null) ? "null result" : result.toString());
                resultPacket.put("closestSwatch", (detectedColor == null) ? "null" : detectedColor.toString());
                dashboard.sendTelemetryPacket(resultPacket);
            }
        } else {
            // Optional: send a light heartbeat so you can see scanner idle state if you want
            TelemetryPacket idle = new TelemetryPacket();
            idle.put("scanning", false);
            dashboard.sendTelemetryPacket(idle);
        }
    }

    // Reset for the next scan
    public void reset() {
        colorReady = false;
        detectedColor = null;
        scanning = false;
    }

    // Converts a Swatch to the corresponding ball character
    public static char toBallChar(PredominantColorProcessor.Swatch swatch) {
        if (swatch == null) return ' ';
        switch (swatch) {
            case ARTIFACT_PURPLE: return 'P';
            case ARTIFACT_GREEN: return 'G';
            default:
                throw new IllegalStateException("Unhandled Color");
        }
    }

}
