package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOmniOpMode_Linear;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp

public class test extends LinearOpMode {
    public Servo kicker = null;
    double tgtPower = 0;

    @Override
    public void runOpMode() {
        kicker = hardwareMap.get(Servo.class, "kicker_servo");
        kicker.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            tgtPower = -gamepad1.left_stick_y;
            telemetry.addData("power", tgtPower);

            if (gamepad1.x) {
                kicker.setPosition(0);
            } else if (gamepad1.y) {
                kicker.setPosition(1);
            }
        }
    }
}