package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVisionColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Disabled
@Autonomous(name = "Color Sensing Auto", group = "Vision")
public class ColorSensingAuto extends LinearOpMode
{
    @Override
    public void runOpMode()
    {

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                //Change ROI as needed
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE)

                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //Change webcam name to what it is
                .build();

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(portal,30);
        // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n");

            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // Display the Color Sensor result.
            telemetry.addData("Best Match", result.closestSwatch);
            telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                    result.RGB[0], result.RGB[1], result.RGB[2]));
            telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                    result.HSV[0], result.HSV[1], result.HSV[2]));
            telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)",
                    result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Best Match", result.closestSwatch);
            dashboard.sendTelemetryPacket(packet);



            sleep(20);
        }
    }
}
