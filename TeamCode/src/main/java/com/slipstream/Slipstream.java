package com.slipstream;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/17/2026
 */

public class Slipstream {
    public Follower follower;
    public AMPC ampc;
    public VelocityController controller;
    public MecanumKinematics kinematics;
    public SlipstreamConfig config;
    public boolean useSlipstream = true;
    private static final String LOG_PATH = "/sdcard/Slipstream/telemetry_log.csv";
    private PrintWriter logWriter;
    private boolean logInitialized = false;
    private int pathIndex = -1;
    private PathChain lastLoggedChain = null;
    private final double[] loggedTThresholds = {0.0, 0.25, 0.5, 0.75, 1.0};
    private int nextTThresholdIndex = 0;
    private long autoStartNs = 0;
    private long pathStartNs = 0;
    private final List<Double> pathDurations = new ArrayList<>();

    public Slipstream(Follower follower, HardwareMap hardwareMap, SlipstreamConfig config) {
        this.follower = follower;
        this.config = config;
        ampc = new AMPC(follower, config);
        controller = new VelocityController(follower, ampc, config);
        kinematics = new MecanumKinematics(hardwareMap, ampc, controller, config);
        initLogging();
    }

    private void initLogging() {
        try {
            File dir = new File("/sdcard/Slipstream/");
            if (!dir.exists()) dir.mkdirs();
            logWriter = new PrintWriter(new FileWriter(LOG_PATH, false));
            logWriter.println("event,time_s,pathIndex,pathT,x,y,heading,desiredVx,desiredVy,desiredOmega,actualVx,actualVy,actualOmega,sysidVx,sysidVy,sysidOmega");
            logWriter.println("config,maxFwd=" + config.maxSpeedForward + ",maxStrafe=" + config.maxSpeedStrafe + ",maxTurn=" + config.maxTurnRate + ",maxDecel=" + config.maxDecel);
            logWriter.flush();
            logInitialized = true;
            autoStartNs = System.nanoTime();
        } catch (IOException e) {
            android.util.Log.e("Slipstream", "Failed to initialize logging", e);
            logInitialized = false;
        }
    }

    public void update() {
        if (useSlipstream) {
            follower.updatePose();

            PathChain currentChain = follower.getCurrentPathChain();
            if (currentChain != null && currentChain != ampc.getActivePath()) {
                ampc.setActivePath(currentChain);
                onPathChange(currentChain);
            }

            ampc.update();
            controller.velocity();
            kinematics.drive();

            logProgress();

            if (ampc.isPathComplete() && follower.isBusy()) {
                onPathComplete();
                follower.breakFollowing();
            }
        } else {
            follower.update();
            PathChain currentChain = follower.getCurrentPathChain();
            if (currentChain != null && currentChain != lastLoggedChain) {
                onPathChange(currentChain);
            }
            if (!follower.isBusy() && pathStartNs > 0) {
                onPathComplete();
            }
        }
    }

    private void onPathChange(PathChain newChain) {
        if (lastLoggedChain != null) {
            onPathComplete();
        }
        pathIndex++;
        pathStartNs = System.nanoTime();
        lastLoggedChain = newChain;
        nextTThresholdIndex = 0;
        logEvent("path_start");
    }

    private void onPathComplete() {
        if (pathStartNs > 0) {
            double duration = (System.nanoTime() - pathStartNs) / 1e9;
            pathDurations.add(duration);
            logEvent("path_end");
            pathStartNs = 0;
        }
    }

    private void logProgress() {
        if (!logInitialized || ampc.getActivePath() == null) return;
        double currentT = ampc.currentT;
        while (nextTThresholdIndex < loggedTThresholds.length && currentT >= loggedTThresholds[nextTThresholdIndex]) {
            logEvent("progress");
            nextTThresholdIndex++;
        }
    }

    private void logEvent(String event) {
        if (!logInitialized) return;
        try {
            double time = (System.nanoTime() - autoStartNs) / 1e9;
            double actualVx = controller != null ? controller.actualVx : 0;
            double actualVy = controller != null ? controller.actualVy : 0;
            double actualOmega = controller != null ? controller.actualOmega : 0;
            logWriter.println(String.format("%s,%.3f,%d,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f", event, time, pathIndex, ampc.currentT, follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), ampc.desiredVx, ampc.desiredVy, ampc.desiredOmega, actualVx, actualVy, actualOmega, ampc.sysIDRatioVx, ampc.sysIDRatioVy, ampc.sysIDRatioOmega));
            logWriter.flush();
        } catch (Exception e) {
            android.util.Log.e("Slipstream", "Logging error", e);
        }
    }

    public void finish() {
        if (!logInitialized) return;
        try {
            double totalTime = 0;
            for (Double d : pathDurations) totalTime += d;
            logWriter.println("");
            logWriter.println("=== SUMMARY ===");
            logWriter.println("path,duration_s");
            for (int i = 0; i < pathDurations.size(); i++) {
                logWriter.println(i + "," + String.format("%.3f", pathDurations.get(i)));
            }
            logWriter.println("TOTAL," + String.format("%.3f", totalTime));
            logWriter.flush();
            logWriter.close();
            logInitialized = false;
        }  catch (Exception e) {
            android.util.Log.e("Slipstream", "Finish error", e);
        }
    }
}