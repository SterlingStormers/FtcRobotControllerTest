package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kinematics Test", group = "Test")
public class KinematicsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveTrainHardware drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        MPC mpc = new MPC();

        VelocityControllerV2 controller = new VelocityControllerV2(follower, mpc);

        MecanumKinematics kinematics = new MecanumKinematics(drive, mpc, controller);

        telemetry.addLine("Kinematics Test Ready");
        telemetry.addLine("A = forward, B = strafe right, X = rotate CCW");
        telemetry.addLine("Release all buttons = stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.updatePose();
            if (gamepad1.a) {
                mpc.desiredVx = 20;
                mpc.desiredVy = 0;
                mpc.desiredOmega = 0;
                telemetry.addLine("Driving forward at 20 in/s");

            } else if (gamepad1.b) {
                mpc.desiredVx = 0;
                mpc.desiredVy = 20;
                mpc.desiredOmega = 0;
                telemetry.addLine("Strafing at 20 in/s");

            } else if (gamepad1.x) {
                mpc.desiredVx = 0;
                mpc.desiredVy = 0;
                mpc.desiredOmega = 1.5;
                telemetry.addLine("Rotating at 1.5 rad/s");

            } else {
                mpc.desiredVx = 0;
                mpc.desiredVy = 0;
                mpc.desiredOmega = 0;
                telemetry.addLine("Stopped");
            }
            controller.velocity(); //must come first
            kinematics.drive();
            telemetry.update();
        }
    }
}