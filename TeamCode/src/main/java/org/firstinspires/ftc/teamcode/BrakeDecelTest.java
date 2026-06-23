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

    public static double CRUISE_VEL = 61.2; // in/s
    public static double ACCEL_TIME = 1.5; // seconds at cruise before braking
    public static double STOPPED_THRESHOLD = 1.0;  // in/s — considered stopped below this

    private Follower follower;
    private DriveTrainHardware drive;
    private AMPC mpc;
    private VelocityControllerV2 controller;
    private MecanumKinematics kinematics;

    @Override
    public void runOpMode() {
        setup();
        waitForStart();

        double cruiseVel = accelerate();
        double startX = follower.getPose().getX();

        brake();
        stopMotors();
        double brakeDist = Math.abs(follower.getPose().getX() - startX);

        double maxDecel = (cruiseVel * cruiseVel) / (2 * brakeDist);
        report(cruiseVel, brakeDist, maxDecel);
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

        sleep(500);   // Pinpoint IMU calibration window
        follower.updatePose();

        telemetry.addLine("Ready. Need 6+ feet clear ahead.");
        telemetry.update();
    }

    // === Phases ===

    /** Accelerates to CRUISE_VEL, returns measured velocity averaged over last 0.3s. */
    private double accelerate() {
        ElapsedTime t = new ElapsedTime();
        double lastX = follower.getPose().getX();
        double lastT = 0;
        double v = 0;
        double velSum = 0;
        int velCount = 0;

        while (opModeIsActive() && t.seconds() < ACCEL_TIME) {
            follower.updatePose();
            double now = t.seconds();
            double x = follower.getPose().getX();
            if (now > lastT) v = (x - lastX) / (now - lastT);
            lastX = x;
            lastT = now;

            // Average velocity in the last 0.3s to smooth pose-differential noise
            if (now > ACCEL_TIME - 0.3) {
                velSum += v;
                velCount++;
            }

            commandVelocity(CRUISE_VEL);
            showStatus("ACCEL", v);
        }

        return velCount > 0 ? velSum / velCount : v;
    }

    /** Commands zero velocity and waits for the robot to actually stop. */
    private void brake() {
        ElapsedTime t = new ElapsedTime();
        double lastX = follower.getPose().getX();
        double lastT = 0;
        double v = 0;

        while (opModeIsActive() && t.seconds() < 3.0) {
            follower.updatePose();
            double now = t.seconds();
            double x = follower.getPose().getX();
            if (now > lastT) v = (x - lastX) / (now - lastT);
            lastX = x;
            lastT = now;

            commandVelocity(0);
            showStatus("BRAKE", v);

            if (Math.abs(v) < STOPPED_THRESHOLD && now > 0.2) return;
        }
    }

    // === Helpers ===

    private void commandVelocity(double vx) {
        mpc.desiredVx = vx;
        mpc.desiredVy = 0;
        mpc.desiredOmega = 0;
        controller.velocity();
        kinematics.drive();
    }

    private void stopMotors() {
        drive.frontLeftDrive.setPower(0);
        drive.frontRightDrive.setPower(0);
        drive.backLeftDrive.setPower(0);
        drive.backRightDrive.setPower(0);
    }

    private void showStatus(String phase, double velocity) {
        telemetry.addData("phase", phase);
        telemetry.addData("velocity (in/s)", "%.2f", velocity);
        telemetry.update();
    }

    private void report(double cruiseVel, double brakeDist, double maxDecel) {
        while (opModeIsActive()) {
            telemetry.addData("Brake start vel (in/s)", "%.2f", cruiseVel);
            telemetry.addData("Brake distance (in)",    "%.2f", brakeDist);
            telemetry.addData("Measured MAX_DECEL",     "%.1f", maxDecel);
            telemetry.addData("Recommended (75%)",      "%.0f", maxDecel * 0.75);
            telemetry.update();
            sleep(100);
        }
    }
}