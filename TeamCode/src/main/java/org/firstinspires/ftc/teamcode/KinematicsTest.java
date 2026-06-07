package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kinematics Test", group = "Test")
public class KinematicsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveTrainHardware drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        MPC mpc = new MPC();
        MecanumKinematics kinematics = new MecanumKinematics(drive, mpc);

        telemetry.addLine("Kinematics Test Ready");
        telemetry.addLine("A = forward, B = strafe right, X = rotate CCW");
        telemetry.addLine("Release all buttons = stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                kinematics.drive(20, 0, 0);
                telemetry.addLine("Driving forward at 20 in/s");
            } else if (gamepad1.b) {
                kinematics.drive(0, 20, 0);
                telemetry.addLine("Strafing at 20 in/s");
            } else if (gamepad1.x) {
                kinematics.drive(0, 0, 1.5);
                telemetry.addLine("Rotating at 1.5 rad/s");
            } else {
                kinematics.drive(0, 0, 0);
                telemetry.addLine("Stopped");
            }
            telemetry.update();
        }
    }
}