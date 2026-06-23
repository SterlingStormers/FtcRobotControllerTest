package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Brake Decel Test", group = "Test")
@Configurable
public class BrakeDecelTest extends LinearOpMode {

    public static double CRUISE_VEL = 61.0;
    public static double ACCEL_TIME = 1.5;
    public static double STOPPED_THRESHOLD = 1.0;
    public static int    NUM_TRIALS = 3;
    public static double PAUSE_BETWEEN = 1.5;

    private Follower follower;
    private DriveTrainHardware drive;
    private AMPC mpc;
    private VelocityControllerV2 controller;
    private MecanumKinematics kinematics;

    @Override
    public void runOpMode() {
        setup();
        waitForStart();

        double[] decelMeasurements = new double[NUM_TRIALS];

        for (int i = 0; i < NUM_TRIALS; i++) {
            double direction = (i % 2 == 0) ? 1.0 : -1.0;   // alternate fwd/back

            telemetry.addData("trial", (i+1) + "/" + NUM_TRIALS);
            telemetry.addData("direction", direction > 0 ? "FORWARD" : "REVERSE");
            telemetry.update();

            double cruiseVel = accelerate(direction);
            double startX = follower.getPose().getX();
            brake(direction);
            stopMotors();
            sleep(300);   // let robot fully settle before measuring final position
            double brakeDist = Math.abs(follower.getPose().getX() - startX);

            decelMeasurements[i] = (cruiseVel * cruiseVel) / (2 * brakeDist);

            sleep((long)(PAUSE_BETWEEN * 1000));
        }

        report(decelMeasurements);
    }

    // === Setup ===

    private void setup() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        mpc = new AMPC(follower);
        controller = new VelocityControllerV2(follower, mpc);
        kinematics = new MecanumKinematics(drive, mpc, controller);

        sleep(500);
        follower.updatePose();

        telemetry.addLine("Brake Decel Test ready");
        telemetry.addLine(NUM_TRIALS + " trials, alternating forward/reverse");
        telemetry.addLine("~5 ft clearance needed (each direction)");
        telemetry.update();
    }

    // === Phases ===

    /** Accelerates to CRUISE_VEL in the given direction (+1 forward, -1 reverse). */
    private double accelerate(double direction) {
        ElapsedTime t = new ElapsedTime();
        double lastX = follower.getPose().getX();
        double lastT = 0, v = 0, velSum = 0;
        int velCount = 0;

        while (opModeIsActive() && t.seconds() < ACCEL_TIME) {
            follower.updatePose();
            double now = t.seconds();
            double x = follower.getPose().getX();
            if (now > lastT) v = (x - lastX) / (now - lastT);
            lastX = x;
            lastT = now;

            if (now > ACCEL_TIME - 0.3) { velSum += v; velCount++; }

            mpc.desiredVx = CRUISE_VEL * direction;
            mpc.desiredVy = 0;
            mpc.desiredOmega = 0;
            controller.velocity();
            kinematics.drive();

            telemetry.addData("phase", "ACCEL " + (direction > 0 ? "FWD" : "REV"));
            telemetry.addData("v (in/s)", "%.2f", v);
            telemetry.update();
        }

        // Return signed velocity, but the measured magnitude is what matters
        return velCount > 0 ? Math.abs(velSum / velCount) : Math.abs(v);
    }

    /** Brakes by commanding motors *opposite* to current motion. */
    private void brake(double direction) {
        ElapsedTime t = new ElapsedTime();
        double lastX = follower.getPose().getX();
        double lastT = 0, v = 999;
        // Brake direction is opposite to motion direction
        double brakePower = -direction;

        while (opModeIsActive() && t.seconds() < 3.0) {
            follower.updatePose();
            double now = t.seconds();
            double x = follower.getPose().getX();
            if (now > lastT) v = (x - lastX) / (now - lastT);
            lastX = x;
            lastT = now;

            drive.frontLeftDrive.setPower(brakePower);
            drive.frontRightDrive.setPower(brakePower);
            drive.backLeftDrive.setPower(brakePower);
            drive.backRightDrive.setPower(brakePower);

            telemetry.addData("phase", "BRAKE");
            telemetry.addData("v (in/s)", "%.2f", v);
            telemetry.update();

            // Exit when velocity sign flips relative to motion direction (robot stopped or reversing)
            if (v * direction <= STOPPED_THRESHOLD && t.seconds() > 0.2) return;
        }
    }

    // === Helpers ===

    private void stopMotors() {
        drive.frontLeftDrive.setPower(0);
        drive.frontRightDrive.setPower(0);
        drive.backLeftDrive.setPower(0);
        drive.backRightDrive.setPower(0);
    }

    private void report(double[] measurements) {
        double min = measurements[0], max = measurements[0], sum = 0;
        for (double m : measurements) {
            min = Math.min(min, m);
            max = Math.max(max, m);
            sum += m;
        }
        double mean = sum / measurements.length;
        double recommended = min * 0.95;

        while (opModeIsActive()) {
            telemetry.addLine("=== Brake Decel Test Results ===");
            for (int i = 0; i < measurements.length; i++) {
                telemetry.addData("trial " + (i+1), "%.1f in/s²", measurements[i]);
            }
            telemetry.addLine();
            telemetry.addData("Min",  "%.1f", min);
            telemetry.addData("Max",  "%.1f", max);
            telemetry.addData("Mean", "%.1f", mean);
            telemetry.addLine();
            telemetry.addData("==> MAX_DECEL", "%.0f", recommended);
            telemetry.addLine("Plug this into AMPC.java");
            telemetry.update();
            sleep(100);
        }
    }
}