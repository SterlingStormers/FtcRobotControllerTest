package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PIDF Step Test", group = "Tuning")
public class PIDFStepTest extends LinearOpMode {
    private static final double TARGET_VELOCITY = 30.0;   // in/s — adjust per test
    private static final double STEP_DURATION_MS = 2000;  // how long to hold the step
    private static final double SAMPLE_INTERVAL_MS = 50;  // log every 50ms
    private static final int AXIS = 0;   // 0 = forward, 1 = strafe, 2 = turn

    @Override
    public void runOpMode() {
        DriveTrainHardware drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        AMPC mpc = new AMPC(follower);
        VelocityControllerV2 controller = new VelocityControllerV2(follower, mpc);
        MecanumKinematics kinematics = new MecanumKinematics(drive, mpc, controller);

        telemetry.addLine("PIDF Step Test ready");
        telemetry.addLine("Axis: " + axisName(AXIS) + ", Target: " + TARGET_VELOCITY);
        telemetry.addLine("Press START — robot will step to target, then stop");
        telemetry.update();

        waitForStart();

        // Step input — command the target velocity
        setDesiredForAxis(mpc, TARGET_VELOCITY);

        long startTime = System.currentTimeMillis();
        long lastSample = 0;

        // Log data structure: time, actual, effort
        StringBuilder log = new StringBuilder();
        log.append("time_ms,desired,actual,effort\n");

        double maxOvershoot = 0;
        double riseTime90 = -1;
        double settleTime = -1;
        long lastUnsettleTime = startTime;

        while (opModeIsActive() && System.currentTimeMillis() - startTime < STEP_DURATION_MS) {
            follower.updatePose();
            controller.velocity();
            kinematics.drive();

            long elapsed = System.currentTimeMillis() - startTime;

            double actual = getActualForAxis(controller, AXIS);
            double effort = getEffortForAxis(controller, AXIS);

            // Sample log
            if (elapsed - lastSample >= SAMPLE_INTERVAL_MS) {
                log.append(elapsed).append(",")
                        .append(TARGET_VELOCITY).append(",")
                        .append(actual).append(",")
                        .append(effort).append("\n");
                lastSample = elapsed;
            }

            // Track overshoot
            double overshoot = actual - TARGET_VELOCITY;
            if (overshoot > maxOvershoot) maxOvershoot = overshoot;

            // Track rise time (first time actual reaches 90% of target)
            if (riseTime90 < 0 && actual >= 0.9 * TARGET_VELOCITY) {
                riseTime90 = elapsed;
            }

            // Track settling (last time actual was outside 2% band)
            if (Math.abs(actual - TARGET_VELOCITY) > 0.02 * TARGET_VELOCITY) {
                lastUnsettleTime = System.currentTimeMillis();
            }

            // Live telemetry
            telemetry.addData("elapsed ms", elapsed);
            telemetry.addData("desired", TARGET_VELOCITY);
            telemetry.addData("actual", actual);
            telemetry.addData("effort", effort);
            telemetry.update();
        }

        // Stop the robot
        setDesiredForAxis(mpc, 0);
        controller.velocity();
        kinematics.drive();

        settleTime = lastUnsettleTime - startTime;

        // Final results screen
        while (opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addLine("=== STEP RESPONSE RESULTS ===");
            telemetry.addData("Target", "%.1f in/s", TARGET_VELOCITY);
            telemetry.addData("Rise time (ms)", "%.0f", riseTime90);
            telemetry.addData("Max overshoot", "%.2f in/s (%.1f%%)", maxOvershoot, 100 * maxOvershoot / TARGET_VELOCITY);
            telemetry.addData("Settle time (ms)", "%.0f", settleTime);
            telemetry.addLine("");
            telemetry.addLine("Press STOP when done");
            telemetry.update();
            sleep(200);
        }
    }

    private void setDesiredForAxis(AMPC mpc, double target) {
        if (AXIS == 0) { mpc.desiredVx = target; mpc.desiredVy = 0; mpc.desiredOmega = 0; }
        else if (AXIS == 1) { mpc.desiredVx = 0; mpc.desiredVy = target; mpc.desiredOmega = 0; }
        else { mpc.desiredVx = 0; mpc.desiredVy = 0; mpc.desiredOmega = target; }
    }

    private double getActualForAxis(VelocityControllerV2 c, int axis) {
        return axis == 0 ? c.actualVx : axis == 1 ? c.actualVy : c.actualOmega;
    }

    private double getEffortForAxis(VelocityControllerV2 c, int axis) {
        return axis == 0 ? c.effortVx : axis == 1 ? c.effortVy : c.effortOmega;
    }

    private String axisName(int axis) {
        return axis == 0 ? "Forward" : axis == 1 ? "Strafe" : "Turn";
    }
}