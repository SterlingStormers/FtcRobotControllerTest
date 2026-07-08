package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Physical Limits Test", group = "SysID")
public class PhysicalLimitsTest extends LinearOpMode {

    private Follower follower;
    private DriveTrainHardware drive;
    private ElapsedTime timer = new ElapsedTime();

    private static final double ACCEL_TEST_DURATION = 1.5;
    private static final double LAT_TEST_RADIUS = 12.0;
    private static final double LAT_TEST_SPEED_START = 15.0;
    private static final double LAT_TEST_SPEED_STEP = 3.0;
    private static final double LAT_TEST_LEVEL_DURATION = 3.0;
    private static final int LAT_TEST_MAX_LEVELS = 15;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24, 24, 0));
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        telemetry.addLine("=== PHYSICAL LIMITS TEST ===");
        telemetry.addLine("");
        telemetry.addLine("Press A = Max Forward Acceleration Test");
        telemetry.addLine("Press B = Max Lateral Acceleration Test (circle)");
        telemetry.addLine("");
        telemetry.addLine("Ensure open space:");
        telemetry.addLine("  A test: 6+ feet in front");
        telemetry.addLine("  B test: 4+ feet all around");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                runAccelTest();
                waitForButtonRelease();
            }
            if (gamepad1.b) {
                runLateralTest();
                waitForButtonRelease();
            }

            telemetry.addLine("Press A (accel) or B (lateral)");
            telemetry.update();
            sleep(50);
        }
    }

    private void waitForButtonRelease() {
        while (opModeIsActive() && (gamepad1.a || gamepad1.b)) {
            sleep(50);
        }
    }

    private void runAccelTest() {
        telemetry.clearAll();
        telemetry.addLine("Acceleration test starting in 2s...");
        telemetry.addLine("Robot will drive forward at full power");
        telemetry.update();
        sleep(2000);

        follower.setStartingPose(new Pose(24, 24, 0));
        follower.updatePose();

        double[] times = new double[200];
        double[] velocities = new double[200];
        int samples = 0;
        double peakVelocity = 0;
        double peakAccel = 0;
        double previousV = 0;
        double previousT = 0;

        timer.reset();

        drive.frontLeftDrive.setPower(1.0);
        drive.frontRightDrive.setPower(1.0);
        drive.backLeftDrive.setPower(1.0);
        drive.backRightDrive.setPower(1.0);

        while (opModeIsActive() && timer.seconds() < ACCEL_TEST_DURATION) {
            follower.updatePose();

            double t = timer.seconds();
            double vx = follower.getVelocity().getXComponent();
            double vy = follower.getVelocity().getYComponent();
            double v = Math.sqrt(vx * vx + vy * vy);

            if (samples < 200) {
                times[samples] = t;
                velocities[samples] = v;
                samples++;
            }

            if (v > peakVelocity) peakVelocity = v;

            if (previousT > 0) {
                double dt = t - previousT;
                if (dt > 0.01) {
                    double accel = (v - previousV) / dt;
                    if (accel > peakAccel && v < peakVelocity * 0.95) peakAccel = accel;
                }
            }

            previousT = t;
            previousV = v;

            telemetry.addData("Time", "%.2f", t);
            telemetry.addData("Velocity", "%.1f in/s", v);
            telemetry.addData("Peak accel", "%.1f in/s²", peakAccel);
            telemetry.update();

            sleep(20);
        }

        drive.frontLeftDrive.setPower(0);
        drive.frontRightDrive.setPower(0);
        drive.backLeftDrive.setPower(0);
        drive.backRightDrive.setPower(0);

        // Compute sustained acceleration during ramp-up phase
        double sustainedAccel = 0;
        int count = 0;
        for (int i = 3; i < samples - 3; i++) {
            if (velocities[i] < peakVelocity * 0.7 && velocities[i] > 5) {
                double dt = times[i+1] - times[i];
                if (dt > 0.01) {
                    double a = (velocities[i+1] - velocities[i]) / dt;
                    if (a > 0) {
                        sustainedAccel += a;
                        count++;
                    }
                }
            }
        }
        if (count > 0) sustainedAccel /= count;

        telemetry.clearAll();
        telemetry.addLine("=== ACCEL RESULTS ===");
        telemetry.addData("Peak instantaneous accel", "%.1f in/s²", peakAccel);
        telemetry.addData("Sustained ramp-up accel", "%.1f in/s²", sustainedAccel);
        telemetry.addData("Peak velocity reached", "%.1f in/s", peakVelocity);
        telemetry.addLine("");
        telemetry.addLine("Use SUSTAINED value for MAX_ACCEL");
        telemetry.addLine("Waiting 8 seconds...");
        telemetry.update();

        sleep(8000);
    }

    private void runLateralTest() {
        telemetry.clearAll();
        telemetry.addLine("Lateral accel test starting in 3s...");
        telemetry.addLine("Robot will drive in " + LAT_TEST_RADIUS + "\" radius circle");
        telemetry.addLine("Speed increases every " + LAT_TEST_LEVEL_DURATION + "s");
        telemetry.addLine("Press X when robot slips outward");
        telemetry.update();
        sleep(3000);

        double currentSpeed = LAT_TEST_SPEED_START;
        int level = 0;
        boolean slipDetected = false;
        double slipSpeed = 0;

        while (opModeIsActive() && !slipDetected && level < LAT_TEST_MAX_LEVELS) {
            level++;
            double omega = currentSpeed / LAT_TEST_RADIUS;

            telemetry.clearAll();
            telemetry.addLine(String.format("Level %d", level));
            telemetry.addData("Speed", "%.1f in/s", currentSpeed);
            telemetry.addData("Omega", "%.2f rad/s", omega);
            telemetry.addData("Lat accel", "%.1f in/s²", currentSpeed * currentSpeed / LAT_TEST_RADIUS);
            telemetry.addLine("");
            telemetry.addLine("Press X if robot slips outward");
            telemetry.update();

            timer.reset();

            while (opModeIsActive() && timer.seconds() < LAT_TEST_LEVEL_DURATION) {
                double powerFwd = currentSpeed / 61.2;
                double powerTurn = omega / 4.0;

                double fl = powerFwd + powerTurn;
                double fr = powerFwd - powerTurn;
                double bl = powerFwd + powerTurn;
                double br = powerFwd - powerTurn;

                double maxMag = Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br))));
                if (maxMag > 1.0) {
                    fl /= maxMag;
                    fr /= maxMag;
                    bl /= maxMag;
                    br /= maxMag;
                }

                drive.frontLeftDrive.setPower(fl);
                drive.frontRightDrive.setPower(fr);
                drive.backLeftDrive.setPower(bl);
                drive.backRightDrive.setPower(br);

                if (gamepad1.x) {
                    slipDetected = true;
                    slipSpeed = currentSpeed;
                    break;
                }

                sleep(20);
            }

            if (!slipDetected) {
                currentSpeed += LAT_TEST_SPEED_STEP;
            }
        }

        drive.frontLeftDrive.setPower(0);
        drive.frontRightDrive.setPower(0);
        drive.backLeftDrive.setPower(0);
        drive.backRightDrive.setPower(0);

        telemetry.clearAll();
        telemetry.addLine("=== LATERAL ACCEL RESULTS ===");
        if (slipDetected) {
            double latAccel = slipSpeed * slipSpeed / LAT_TEST_RADIUS;
            telemetry.addData("Slip at speed", "%.1f in/s", slipSpeed);
            telemetry.addData("Radius", "%.1f in", LAT_TEST_RADIUS);
            telemetry.addData("MAX_LAT_ACCEL", "%.1f in/s²", latAccel);
        } else {
            double maxLat = currentSpeed * currentSpeed / LAT_TEST_RADIUS;
            telemetry.addData("No slip up to", "%.1f in/s", currentSpeed);
            telemetry.addData("MAX_LAT_ACCEL >=", "%.1f in/s²", maxLat);
        }
        telemetry.update();
        sleep(15000);
    }
}