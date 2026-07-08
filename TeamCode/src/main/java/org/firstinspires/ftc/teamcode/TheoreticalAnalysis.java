package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Theoretical Analysis", group = "Benchmark")
public class TheoreticalAnalysis extends LinearOpMode {

    // Physical robot parameters — plug in measured values here
    public static double MAX_ACCEL = 156.7;        // in/s² (from PhysicalLimitsTest - A test)
    public static double MAX_DECEL = 244;        // in/s² (from BrakeDecelTest)
    public static double MAX_LAT_ACCEL = 300;     // in/s² (from PhysicalLimitsTest - B test)
    public static double MAX_SPEED = 61.2;       // in/s (from ForwardVelocityTuner)

    public static double PATH_BUFFER = 0.3;      // seconds between paths (state machine timer)

    private static final int N_SAMPLES = 200;

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(57.724, 37.697),
                                    new Pose(39.203, 35.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(39.203, 35.300),
                                    new Pose(12.694, 35.464)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.694, 35.464),
                                    new Pose(32.338, 35.974),
                                    new Pose(51.160, 29.975),
                                    new Pose(47.808, 59.380)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.808, 59.380),
                                    new Pose(12.576, 59.243)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        Paths paths = new Paths(follower);

        waitForStart();

        // Compute per-path theoretical minimums
        double t1 = theoreticalMinTime(paths.Path1);
        double t2 = theoreticalMinTime(paths.Path2);
        double t3 = theoreticalMinTime(paths.Path3);
        double t4 = theoreticalMinTime(paths.Path4);

        double sumPaths = t1 + t2 + t3 + t4;
        double totalWithBuffer = sumPaths + 3 * PATH_BUFFER;   // 3 transitions between 4 paths

        while (opModeIsActive()) {
            telemetry.addLine("=== THEORETICAL MINIMUM TIMES ===");
            telemetry.addData("Path 1", "%.3fs  (%.1f in, curve)", t1, paths.Path1.length());
            telemetry.addData("Path 2", "%.3fs  (%.1f in, line)", t2, paths.Path2.length());
            telemetry.addData("Path 3", "%.3fs  (%.1f in, cubic curve)", t3, paths.Path3.length());
            telemetry.addData("Path 4", "%.3fs  (%.1f in, line)", t4, paths.Path4.length());
            telemetry.addLine("");
            telemetry.addData("Sum of paths", "%.3fs", sumPaths);
            telemetry.addData("With 0.3s buffers × 3", "%.3fs", totalWithBuffer);
            telemetry.addLine("");
            telemetry.addLine("=== PARAMETERS ===");
            telemetry.addData("MAX_ACCEL", "%.1f in/s²", MAX_ACCEL);
            telemetry.addData("MAX_DECEL", "%.1f in/s²", MAX_DECEL);
            telemetry.addData("MAX_LAT_ACCEL", "%.1f in/s²", MAX_LAT_ACCEL);
            telemetry.addData("MAX_SPEED", "%.1f in/s", MAX_SPEED);
            telemetry.update();
            sleep(200);
        }
    }

    /**
     * Compute theoretical minimum time to traverse a path.
     * Respects motor speed, acceleration, deceleration, and lateral accel limits.
     */
    public static double theoreticalMinTime(PathChain chain) {
        Path path = chain.getPath(0);
        double pathLen = chain.length();

        if (pathLen < 0.01) return 0;

        double ds = pathLen / N_SAMPLES;
        double[] vMax = new double[N_SAMPLES + 1];
        double[] v = new double[N_SAMPLES + 1];

        // 1. Compute max feasible speed at each point (curvature + motor limit)
        for (int i = 0; i <= N_SAMPLES; i++) {
            double t = (double) i / N_SAMPLES;
            double curvature = Math.abs(path.getCurvature(t));

            double vCurve = (curvature > 0.0001)
                    ? Math.sqrt(MAX_LAT_ACCEL / curvature)
                    : Double.MAX_VALUE;

            vMax[i] = Math.min(vCurve, MAX_SPEED);
        }

        // 2. Forward pass: acceleration limit from v = 0 at start
        v[0] = 0;
        for (int i = 1; i <= N_SAMPLES; i++) {
            double vAccel = Math.sqrt(v[i-1] * v[i-1] + 2 * MAX_ACCEL * ds);
            v[i] = Math.min(vMax[i], vAccel);
        }

        // 3. Backward pass: deceleration limit to v = 0 at endpoint
        v[N_SAMPLES] = 0;
        for (int i = N_SAMPLES - 1; i >= 0; i--) {
            double vDecel = Math.sqrt(v[i+1] * v[i+1] + 2 * MAX_DECEL * ds);
            v[i] = Math.min(v[i], vDecel);
        }

        // 4. Integrate 1/v ds to get total time
        double totalTime = 0;
        for (int i = 0; i < N_SAMPLES; i++) {
            double vAvg = (v[i] + v[i+1]) / 2;
            if (vAvg > 0.01) {
                totalTime += ds / vAvg;
            }
        }

        return totalTime;
    }
}