package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

public class AMPCV2 {
    private final Follower follower;

    public double maxSpeedForward = 61.2;
    public double maxSpeedStrafe = 45.5;
    public double maxTurnRateRad = 4.0;

    public double desiredVx = 0;
    public double desiredVy = 0;
    public double desiredOmega = 0;

    private PathChain activePath = null;
    public double currentT = 0;

    // Path-search resolution
    private static final int T_COARSE_STEPS = 50;
    private static final int T_FINE_STEPS = 40;
    private static final double FINE_WINDOW = 0.04;
    private static final double HEADING_GAIN = 2;

    public PathChain getActivePath() { return activePath; }
    private static final double LOOKAHEAD_T_DELTA = 0.1;
    public double lookaheadT = 0;

    // === MULTI-STEP HORIZON CONFIG ===
    private static final double STEP_DT = 0.1;           // seconds per rollout step
    private static final int HORIZON_STEPS = 5;          // total horizon = STEP_DT * HORIZON_STEPS = 0.5s
    private static final int GRID_HALF = 1;              // 3×3×3 = 27 candidates
    private static final double GRID_STEP_FRACTION = 0.2;

    // Cost weights — applied per-step, accumulated over rollout
    private static final double WEIGHT_LOOKAHEAD = 1.0;
    private static final double WEIGHT_PATH = 0.5;
    private static final double WEIGHT_HEADING = 10.0;

    private double lastBestVx = 0;
    private double lastBestVy = 0;
    private double lastBestOmega = 0;
    public double lastBestCost = 0;
    public Pose lookaheadPose = new Pose(0, 0, 0);
    public double pursuitVx = 0;
    public double pursuitVy = 0;
    public double pursuitOmega = 0;

    private boolean firstLoop = true;
    private static final double MAX_DECEL = 183;
    private static final double WEIGHT_TERMINAL = 100;
    public boolean terminalTriggered = false;
    private static final double PATH_END_TOLERANCE = 1.5;

    // Path length cache (computed once per setActivePath)
    private double pathLengthInches = 1.0;
    private double predictedX = 0;
    private double predictedY = 0;

    public boolean isPathComplete() {
        if (activePath == null) return true;
        if (currentT >= 0.95) return true;
        Pose robotPose = follower.getPose();
        Pose end = activePath.getPath(0).getPose(1.0); // will need to be changed from 0 for people with multiple paths in 1 path chain
        double dx = robotPose.getX() - end.getX();
        double dy = robotPose.getY() - end.getY();
        return Math.sqrt(dx * dx + dy * dy) < PATH_END_TOLERANCE;

    }
    public AMPCV2(Follower follower) {
        this.follower = follower;
    }
    public void setActivePath(PathChain path) {
        activePath = path;
        currentT = 0;
        firstLoop = true;

        // Estimate path length for converting predicted velocity to t-advancement during rollout
        // Sample 20 points along the path, sum distances
        double length = 0;
        Pose previous = activePath.getPath(0).getPose(0); // will need to be changed from 0 for people with multiple paths in 1 path chain
        for (int i = 1; i <= 20; i++) {
            Pose current = activePath.getPath(0).getPose(i / 20.0); // will need to be changed from 0 for people with multiple paths in 1 path chain
            double dx = current.getX() - previous.getX();
            double dy = current.getY() - previous.getY();
            length = length + Math.sqrt((dx * dx) + (dy * dy));
            previous = current;
        }
        pathLengthInches = Math.max(1.0, length);   // floor to avoid divide-by-zero
    }
    public void updateClosestT() {
        if (activePath != null) {

            Pose currentPose = follower.getPose();
            double bestT = 0;
            double bestDist = Double.MAX_VALUE;

            for (int i = 0; i <= T_COARSE_STEPS; i++) {
                double t = (double) i / T_COARSE_STEPS;
                Pose samplePose = activePath.getPath(0).getPose(t); // will need to be changed from 0 for people with multiple paths in 1 path chain
                double dx = samplePose.getX() - currentPose.getX();
                double dy = samplePose.getY() - currentPose.getY();
                double dist = (dx * dx) + (dy * dy);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestT = t;
                }
            }

            double tMin = Math.max(0.0, bestT - FINE_WINDOW);
            double tMax = Math.min(1.0, bestT + FINE_WINDOW);
            for (int i = 0; i <= T_FINE_STEPS; i++) {
                double t = tMin + ((tMax - tMin) * ((double) i / T_FINE_STEPS));
                Pose samplePose = activePath.getPath(0).getPose(t); // will need to be changed from 0 for people with multiple paths in 1 path chain
                double dx = samplePose.getX() - currentPose.getX();
                double dy = samplePose.getY() - currentPose.getY();
                double dist = (dx * dx) + (dy * dy);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestT = t;
                }
            }
            currentT = bestT;
        }
    }
    public void updateLookahead() {
        if (activePath != null) {
            lookaheadT = Math.min(1.0, currentT + LOOKAHEAD_T_DELTA);
            lookaheadPose = activePath.getPath(0).getPose(lookaheadT); // will need to be changed from 0 for people with multiple paths in 1 path chain
        }
    }
    public void updateMPC() {
        if (activePath == null) {
            desiredVx = 0;
            desiredVy = 0;
            desiredOmega = 0;
            return;
        }

        Pose robotPose = follower.getPose();

        if (firstLoop) {
            computePurePursuit(robotPose);
            lastBestVx = pursuitVx;
            lastBestVy = pursuitVy;
            lastBestOmega = pursuitOmega;
            firstLoop = false;
        }

        double vxStep = GRID_STEP_FRACTION * maxSpeedForward;
        double vyStep = GRID_STEP_FRACTION * maxSpeedStrafe;
        double omegaStep = GRID_STEP_FRACTION * maxTurnRateRad;

        double bestCost = Double.MAX_VALUE;
        double bestVx = lastBestVx;
        double bestVy = lastBestVy;
        double bestOmega = lastBestOmega;

        // MULTISTEP GRID SEARCH
        for (int i = -GRID_HALF; i <= GRID_HALF; i++) {
            for (int j = -GRID_HALF; j <= GRID_HALF; j++) {
                for (int k = -GRID_HALF; k <= GRID_HALF; k++) {
                    double candVx = clamp((lastBestVx + i) * vxStep, -maxSpeedForward, maxSpeedForward);
                    double candVy = clamp((lastBestVy + j) * vyStep, -maxSpeedStrafe, maxSpeedStrafe);
                    double candOmega = clamp((lastBestOmega + k) * omegaStep, -maxTurnRateRad, maxTurnRateRad);

                    double cost = rolloutCost(candVx, candVy, candOmega, robotPose);
                    if (cost < bestCost) {
                        bestCost = cost;
                        bestVx = candVx;
                        bestVy = candVy;
                        bestOmega = candOmega;
                    }
                }
            }
        }

        // Pure pursuit escape candidate (helps grid search find new basins)
        computePurePursuit(robotPose);
        double ppCost = evaluateCandidates(pursuitVx, pursuitVy, pursuitOmega, robotPose);
        if (ppCost < bestCost) {
            bestCost = ppCost;
            bestVx = pursuitVx;
            bestVy = pursuitVy;
            bestOmega = pursuitOmega;
        }

        desiredVx = bestVx;
        desiredVy = bestVy;
        desiredOmega = bestOmega;
        lastBestVx = bestVx;
        lastBestVy = bestVy;
        lastBestOmega = bestOmega;
        lastBestCost = bestCost;
    }
    private double evaluateCandidates(double vx, double vy, double omega, Pose startPose) {
        double currentSimulatedX = startPose.getX();
        double currentSimulatedY = startPose.getY();
        double heading = startPose.getHeading();
        double predictedT = currentT;

        // Estimate how much t advances per step given this candidate's translational speed
        // (path-parameter increments by (distance_traveled / path_length) per step)
        double speed = Math.sqrt((vx * vx) + (vy * vy));
        double tAdvancePerStep = (speed * STEP_DT) / pathLengthInches;

        double totalCost = 0;
        terminalTriggered = false;

        for (int step = 1; step <= HORIZON_STEPS; step++) {
            // Forward-simulate one step
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double fieldVx = (vx * cosH) - (vy * sinH);
            double fieldVy = (vx * sinH) + (vy * cosH);
            predictedX = currentSimulatedX + (fieldVx * STEP_DT);
            predictedY = currentSimulatedY + (fieldVy * STEP_DT);
            currentSimulatedX = predictedX;
            currentSimulatedY = predictedY; // pick up from here
            heading += omega * STEP_DT;
            predictedT = Math.min(1.0, predictedT + tAdvancePerStep);

            // Per-step cost: distance from predicted position to path point at advanced tParam
            Pose pathPointAtT = activePath.getPath(0).getPose(predictedT);
            double dxPath = pathPointAtT.getX() - predictedX;
            double dyPath = pathPointAtT.getY() - predictedY;
            double distPath = Math.sqrt(dxPath * dxPath + dyPath * dyPath);

            // Per-step cost: heading error against path heading at this t
            double headingErr = Math.abs(wrapAngle(pathPointAtT.getHeading() - heading));

            // Lookahead cost still uses the original lookahead pose (the "where am I heading" point)
            double dxLook = lookaheadPose.getX() - predictedX;
            double dyLook = lookaheadPose.getY() - predictedY;
            double distLook = Math.sqrt(dxLook * dxLook + dyLook * dyLook);

            totalCost += WEIGHT_LOOKAHEAD * distLook + WEIGHT_PATH * distPath + WEIGHT_HEADING * headingErr;
        }

        // Terminal cost: can robot stop in time before path end?
        Pose endPose = activePath.getPath(0).getPose(1.0);
        double dxEnd = endPose.getX() - predictedX;
        double dyEnd = endPose.getY() - predictedY;
        double remainingDist = Math.sqrt(dxEnd * dxEnd + dyEnd * dyEnd);
        double brakeDist = (speed * speed) / (2.0 * MAX_DECEL);

        if (brakeDist > remainingDist) {
            totalCost += WEIGHT_TERMINAL * (brakeDist - remainingDist);
            terminalTriggered = true;
        }

        return totalCost;
    }

}
