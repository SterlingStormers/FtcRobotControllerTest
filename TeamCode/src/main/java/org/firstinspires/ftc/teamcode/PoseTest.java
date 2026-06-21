package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Drift and Hold Test", group = "Test")
public class PoseTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrainHardware drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        AMPC mpc = new AMPC(follower);
        VelocityControllerV2 controller = new VelocityControllerV2(follower, mpc);
        MecanumKinematics kinematics = new MecanumKinematics(drive, mpc, controller);

        // Pull the first available voltage sensor (covers all FTC hub configs)
        VoltageSensor voltage = hardwareMap.voltageSensor.iterator().next();

        // Phase 1: let Pinpoint IMU calibrate before play.
        // Robot MUST be stationary here.
        telemetry.addLine("CALIBRATING IMU — DO NOT TOUCH THE ROBOT");
        telemetry.update();
        sleep(1000);
        follower.updatePose();

        // Capture init heading as reference for drift measurement
        double initHeading = follower.getPose().getHeading();

        telemetry.addLine("Init complete. Press play.");
        telemetry.addData("Init heading deg", Math.toDegrees(initHeading));
        telemetry.update();
        waitForStart();

        // Hold gains
        double KP_POS = 5.0;
        double KP_HEADING = 5.0;

        double minVoltage = voltage.getVoltage();
        long startTime = System.nanoTime();

        while (opModeIsActive()) {
            follower.updatePose();
            Pose p = follower.getPose();

            double elapsedSec = (System.nanoTime() - startTime) / 1e9;
            double heading = p.getHeading();
            double headingDriftDeg = Math.toDegrees(heading - initHeading);

            // Voltage tracking — catches brown-outs even brief ones
            double batteryVolts = voltage.getVoltage();
            if (batteryVolts < minVoltage) minVoltage = batteryVolts;

            // Active hold: command return to (0, 0, 0)
            double errorXField = -p.getX();
            double errorYField = -p.getY();
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double cmdVx = (errorXField * cosH + errorYField * sinH) * KP_POS;
            double cmdVy = (-errorXField * sinH + errorYField * cosH) * KP_POS;
            double cmdOmega = -KP_HEADING * heading;

            mpc.desiredVx = cmdVx;
            mpc.desiredVy = cmdVy;
            mpc.desiredOmega = cmdOmega;

            controller.velocity();
            kinematics.drive();

            telemetry.addData("X", p.getX());
            telemetry.addData("Y", p.getY());
            telemetry.addData("heading deg", Math.toDegrees(heading));
            telemetry.addData("heading drift deg (since init)", headingDriftDeg);
            if (elapsedSec > 0.5) {
                telemetry.addData("drift rate deg/s", headingDriftDeg / elapsedSec);
            }
            telemetry.addData("battery V (now)", batteryVolts);
            telemetry.addData("battery V (lowest seen)", minVoltage);
            telemetry.update();
        }
    }
}