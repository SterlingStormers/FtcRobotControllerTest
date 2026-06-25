package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


public class AMPCV1 {
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


    public boolean isPathComplete() {
        if (activePath == null) return true;
        if (currentT < 0.95) return false;


        Pose robot = follower.getPose();
        Pose end = activePath.getPath(0).getPose(1.0);
        double dx = robot.getX() - end.getX();
        double dy = robot.getY() - end.getY();
        return Math.sqrt(dx * dx + dy * dy) < PATH_END_TOLERANCE;
    }


    public AMPCV1(Follower follower) {
        this.follower = follower;
    }


    public void setActivePath(PathChain path) {
        this.activePath = path;
        this.currentT = 0;
        this.firstLoop = true;


        // Estimate path length for converting predicted velocity to t-advancement during rollout
        // Sample 20 points along the path, sum distances
        double length = 0;
        Pose previous = path.getPath(0).getPose(0);
        for (int i = 1; i <= 20; i++) {
            Pose current = path.getPath(0).getPose(i / 20.0);
            double dx = current.getX() - previous.getX();
            double dy = current.getY() - previous.getY();
            length = length + Math.sqrt(dx * dx + dy * dy);
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
                Pose samplePose = activePath.getPath(0).getPose(t);
                double dx = samplePose.getX() - currentPose.getX();
                double dy = samplePose.getY() - currentPose.getY();
                double dist = dx * dx + dy * dy;
                if (dist < bestDist) {
                    bestDist = dist;
                    bestT = t;
                }
            }


            double tMin = Math.max(0.0, bestT - FINE_WINDOW);
            double tMax = Math.min(1.0, bestT + FINE_WINDOW);
            for (int i = 0; i <= T_FINE_STEPS; i++) {
                double t = tMin + (tMax - tMin) * ((double) i / T_FINE_STEPS);
                Pose samplePose = activePath.getPath(0).getPose(t);
                double dx = samplePose.getX() - currentPose.getX();
                double dy = samplePose.getY() - currentPose.getY();
                double distSq = dx * dx + dy * dy;
                if (distSq < bestDist) {
                    bestDist = distSq;
                    bestT = t;
                }
            }
            currentT = bestT;
        }
    }


    public void updateLookahead() {
        if (activePath == null) return;
        lookaheadT = Math.min(1.0, currentT + LOOKAHEAD_T_DELTA);
        lookaheadPose = activePath.getPath(0).getPose(lookaheadT);
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


        // === MULTI-STEP GRID SEARCH ===
        for (int i = -GRID_HALF; i <= GRID_HALF; i++) {
            for (int j = -GRID_HALF; j <= GRID_HALF; j++) {
                for (int k = -GRID_HALF; k <= GRID_HALF; k++) {
                    double candVx = clamp(lastBestVx + i * vxStep, -maxSpeedForward, maxSpeedForward);
                    double candVy = clamp(lastBestVy + j * vyStep, -maxSpeedStrafe, maxSpeedStrafe);
                    double candOmega = clamp(lastBestOmega + k * omegaStep, -maxTurnRateRad, maxTurnRateRad);


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
        double ppCost = rolloutCost(pursuitVx, pursuitVy, pursuitOmega, robotPose);
        if (ppCost < bestCost) {
            bestCost = ppCost;
            bestVx = pursuitVx;
            bestVy = pursuitVy;
            bestOmega = pursuitOmega;
        }


        // NO brake escape — multi-step horizon picks deceleration naturally


        desiredVx = bestVx;
        desiredVy = bestVy;
        desiredOmega = bestOmega;
        lastBestVx = bestVx;
        lastBestVy = bestVy;
        lastBestOmega = bestOmega;
        lastBestCost = bestCost;
    }


    /**
     * Multi-step rollout. Holds (vx, vy, omega) constant across all steps,
     * simulates forward, accumulates per-step cost, adds terminal cost on final state.
     */
    private double rolloutCost(double vx, double vy, double omega, Pose startPose) {
        double x = startPose.getX();
        double y = startPose.getY();
        double heading = startPose.getHeading();
        double tParam = currentT;


        // Estimate how much t advances per step given this candidate's translational speed
        // (path-parameter increments by (distance_traveled / path_length) per step)
        double speed = Math.sqrt(vx * vx + vy * vy);
        double tAdvancePerStep = (speed * STEP_DT) / pathLengthInches;


        double totalCost = 0;
        terminalTriggered = false;


        for (int step = 1; step <= HORIZON_STEPS; step++) {
            // Forward-simulate one step
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double fieldVx = vx * cosH - vy * sinH;
            double fieldVy = vx * sinH + vy * cosH;
            x += fieldVx * STEP_DT;
            y += fieldVy * STEP_DT;
            heading += omega * STEP_DT;
            tParam = Math.min(1.0, tParam + tAdvancePerStep);


            // Per-step cost: distance from predicted position to path point at advanced tParam
            Pose pathPointAtT = activePath.getPath(0).getPose(tParam);
            double dxPath = pathPointAtT.getX() - x;
            double dyPath = pathPointAtT.getY() - y;
            double distPath = Math.sqrt(dxPath * dxPath + dyPath * dyPath);


            // Per-step cost: heading error against path heading at this t
            double headingErr = Math.abs(wrapAngle(pathPointAtT.getHeading() - heading));


            // Lookahead cost still uses the original lookahead pose (the "where am I heading" point)
            double dxLook = lookaheadPose.getX() - x;
            double dyLook = lookaheadPose.getY() - y;
            double distLook = Math.sqrt(dxLook * dxLook + dyLook * dyLook);


            totalCost += WEIGHT_LOOKAHEAD * distLook + WEIGHT_PATH * distPath + WEIGHT_HEADING * headingErr;
        }


        // Terminal cost: can robot stop in time before path end?
        Pose endPose = activePath.getPath(0).getPose(1.0);
        double dxEnd = endPose.getX() - x;
        double dyEnd = endPose.getY() - y;
        double remainingDist = Math.sqrt(dxEnd * dxEnd + dyEnd * dyEnd);
        double brakeDist = (speed * speed) / (2.0 * MAX_DECEL);


        if (brakeDist > remainingDist) {
            totalCost += WEIGHT_TERMINAL * (brakeDist - remainingDist);
            terminalTriggered = true;
        }


        return totalCost;
    }


    private void computePurePursuit(Pose robotPose) {
        double dxField = lookaheadPose.getX() - robotPose.getX();
        double dyField = lookaheadPose.getY() - robotPose.getY();


        double heading = robotPose.getHeading();
        double dxRobot = dxField * Math.cos(heading) + dyField * Math.sin(heading);
        double dyRobot = -dxField * Math.sin(heading) + dyField * Math.cos(heading);


        double dist = Math.sqrt(dxRobot * dxRobot + dyRobot * dyRobot);
        if (dist < 0.01) {
            pursuitVx = 0;
            pursuitVy = 0;
        } else {
            pursuitVx = (dxRobot / dist) * maxSpeedForward;
            pursuitVy = (dyRobot / dist) * maxSpeedForward;
        }


        double headingError = wrapAngle(lookaheadPose.getHeading() - heading);
        pursuitOmega = clamp(headingError * HEADING_GAIN, -maxTurnRateRad, maxTurnRateRad);
    }


    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }


    private double wrapAngle(double a) {
        while (a > Math.PI)   a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }


    public void update() {
        if (activePath == null) {
            desiredVx = 0;
            desiredVy = 0;
            desiredOmega = 0;
            return;
        }
        updateClosestT();
        updateLookahead();
        updateMPC();
    }
}

