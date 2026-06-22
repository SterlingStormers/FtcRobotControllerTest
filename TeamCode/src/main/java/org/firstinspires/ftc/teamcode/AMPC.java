package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

public class AMPC {
    private final Follower follower;

    // Robot model parameters (will be refined by SysID later)
    public double maxSpeedForward = 54.8;
    public double maxSpeedStrafe = 46.0;
    public double maxTurnRateRad = 4.0;

    // Current desired velocity output (read by VelocityController)
    public double desiredVx = 0;
    public double desiredVy = 0;
    public double desiredOmega = 0;

    // Path state
    private PathChain activePath = null;
    public double currentT = 0;

    // Search resolution for finding closest t (two-pass refinement)
    private static final int T_COARSE_STEPS = 50;     // ~0.02 t resolution over full path
    private static final int T_FINE_STEPS = 40;       // refines to ~0.002 t resolution
    private static final double FINE_WINDOW = 0.04;   // window around coarse best
    private static final double HEADING_GAIN = 2;
    public PathChain getActivePath() { return activePath; }
    // Lookahead configuration
    private static final double LOOKAHEAD_T_DELTA = 0.1;   // ~5 in on 50 in test path
    // Lookahead output (read by Step 3+)
    public double lookaheadT = 0;
    // MPC configuration
    private static final double HORIZON_SECONDS = 0.1;
    private static final int GRID_HALF = 1; // 1 → 3×3×3 = 27 candidates
    private static final double GRID_STEP_FRACTION = 0.2; // ±20% of max per step 0.2
    // Cost weights
    private static final double WEIGHT_LOOKAHEAD = 1.0;
    private static final double WEIGHT_PATH = 0.5;
    private static final double WEIGHT_HEADING = 10.0;   // radians are small; needs big weight

    // State carried across loops (for dynamic grid centering)
    private double lastBestVx = 0;
    private double lastBestVy = 0;
    private double lastBestOmega = 0;
    public double lastBestCost = 0;   // public for telemetry
    public Pose lookaheadPose = new Pose(0, 0, 0);
    public double pursuitVx = 0;
    public double pursuitVy = 0;
    public double pursuitOmega = 0;
    // Smoothness weights — penalize change from last command (squared)
    private static final double WEIGHT_SMOOTHNESS_VX = 0.0;
    private static final double WEIGHT_SMOOTHNESS_VY = 0.0;
    private static final double WEIGHT_SMOOTHNESS_OMEGA = 0.0;   // rad/s scale, needs bigger weight
    private boolean firstLoop = true;
    private static final double MAX_DECEL = 167;     // 100
    private static final double WEIGHT_TERMINAL = 100;  //100
    public double terminalCost = 0;
    public boolean terminalTriggered = false;
    private static final double SCRUBBING_FACTOR = 0.15;
    private static final double PATH_END_TOLERANCE = 1.5;  // inches

    public boolean isPathComplete() {
        if (activePath == null) return true;
        if (currentT < 0.95) return false;

        // Robot must be within tolerance of endpoint
        Pose robot = follower.getPose();
        Pose end = activePath.getPath(0).getPose(1.0);
        double dx = robot.getX() - end.getX();
        double dy = robot.getY() - end.getY();
        return Math.sqrt(dx * dx + dy * dy) < PATH_END_TOLERANCE;
    }

    public AMPC(Follower follower) {
        this.follower = follower;
    }
    public void setActivePath(PathChain path) {
        this.activePath = path;
        this.currentT = 0;
        this.firstLoop = true;
    }

    public void updateClosestT() {
        if (activePath != null) {

        Pose currentPose = follower.getPose();
        double bestT = 0;
        double bestDist = Double.MAX_VALUE;

        // Coarse pass: full path, 50 samples → ~1 in resolution on a 50 in path
        for (int i = 0; i <= T_COARSE_STEPS; i++) {
            double t = (double) i / T_COARSE_STEPS;
            Pose samplePose = activePath.getPath(0).getPose(t);
            double dx = samplePose.getX() - currentPose.getX();
            double dy = samplePose.getY() - currentPose.getY();
            double dist = (dx * dx) + (dy * dy);  // no sqrt — same ordering
            if (dist < bestDist) {
                bestDist = dist;
                bestT = t;
            }
        }
        // Fine pass: 40 samples in ±0.04 around coarse best, clamped to [0,1]
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
        if (activePath != null) {

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
            // This forces the grid to get hyper-fine as the robot slows down near the path
//            double vxStep = GRID_STEP_FRACTION * Math.max(8.0, Math.abs(lastBestVx));
//            double vyStep = GRID_STEP_FRACTION * Math.max(8.0, Math.abs(lastBestVy));
//            double omegaStep = GRID_STEP_FRACTION * Math.max(1.0, Math.abs(lastBestOmega));

            double bestCost = Double.MAX_VALUE;
            double bestVx = lastBestVx;
            double bestVy = lastBestVy;
            double bestOmega = lastBestOmega;

            // Dynamic grid centered on last best command
            for (int i = -GRID_HALF; i <= GRID_HALF; i++) {
                for (int j = -GRID_HALF; j <= GRID_HALF; j++) {
                    for (int k = -GRID_HALF; k <= GRID_HALF; k++) {
                        double candVx = clamp((lastBestVx + (i * vxStep)), -maxSpeedForward, maxSpeedForward);
                        double candVy = clamp((lastBestVy + (j * vyStep)), -maxSpeedStrafe, maxSpeedStrafe);
                        double candOmega = clamp((lastBestOmega + (k * omegaStep)), -maxTurnRateRad, maxTurnRateRad);

                        double cost = evaluateCandidate(candVx, candVy, candOmega, robotPose);
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestVx = candVx;
                            bestVy = candVy;
                            bestOmega = candOmega;
                        }
                    }
                }
            }

            // Global escape candidate 1: pure pursuit
            computePurePursuit(robotPose);
            double ppCost = evaluateCandidate(pursuitVx, pursuitVy, pursuitOmega, robotPose);
            if (ppCost < bestCost) {
                bestCost = ppCost;
                bestVx = pursuitVx;
                bestVy = pursuitVy;
                bestOmega = pursuitOmega;
            }

            // Global escape candidate 2: brake (zero command)
            double brakeCost = evaluateCandidate(0, 0, 0, robotPose);
            if (brakeCost < bestCost) {
                bestCost = brakeCost;
                bestVx = 0;
                bestVy = 0;
                bestOmega = 0;
            }
            // Commit
            desiredVx = bestVx;
            desiredVy = bestVy;
            desiredOmega = bestOmega;
            lastBestVx = bestVx;
            lastBestVy = bestVy;
            lastBestOmega = bestOmega;
            lastBestCost = bestCost;
        } else {
            desiredVx = 0;
            desiredVy = 0;
            desiredOmega = 0;
        }
    }

    private double evaluateCandidate(double vx, double vy, double omega, Pose robotPose) {
        // Convert robot-frame velocity to field frame for prediction
        double heading = robotPose.getHeading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        //Forward speed scrubbing based on turning
//        double scrubbingLossMultiplier = 1.0 - (Math.abs(omega) / maxTurnRateRad) * SCRUBBING_FACTOR;
        double fieldVx = (vx * cosH) - (vy * sinH) /* * scrubbingLossMultiplier*/;
        double fieldVy = (vx * sinH) + (vy * cosH) /* * scrubbingLossMultiplier*/;

        double predictedX = robotPose.getX() + (fieldVx * HORIZON_SECONDS);
        double predictedY = robotPose.getY() + (fieldVy * HORIZON_SECONDS);
        double predictedHeading = heading + (omega * HORIZON_SECONDS);
        // Cost term 1: distance from predicted pose to lookahead
        double dxError = lookaheadPose.getX() - predictedX;
        double dyError = lookaheadPose.getY() - predictedY;
        double distError = Math.sqrt((dxError * dxError) + (dyError * dyError));
        // Cost term 2: distance from predicted pose to closest point on path
        Pose closestPath = activePath.getPath(0).getPose(currentT);
        double dxPathError = closestPath.getX() - predictedX;
        double dyPathError = closestPath.getY() - predictedY;
        double distPathError = Math.sqrt((dxPathError * dxPathError) + (dyPathError * dyPathError));
//        double pathHeading = closestPath.getHeading();
//        double crossTrackError = (-dxPathError * Math.sin(pathHeading)) + (dyPathError * Math.cos(pathHeading));
//        double distPathError = Math.abs(crossTrackError);
        //Cost term 3: heading error
        double headingError = Math.abs(wrapAngle(lookaheadPose.getHeading() - predictedHeading));
        // Cost term 4: smoothness, penalize change from last command
        double dVx = vx - lastBestVx;
        double dVy = vy - lastBestVy;
        double dOmega = omega - lastBestOmega;
        double smoothnessCost = (WEIGHT_SMOOTHNESS_VX * (dVx * dVx)) + (WEIGHT_SMOOTHNESS_VY * (dVy * dVy)) + (WEIGHT_SMOOTHNESS_OMEGA * (dOmega * dOmega));
        //Dynamic endpoint cost
//        double endpointWeightMultiplier = 1.0 + (currentT * currentT * 4.0);
        // Terminal cost: penalize "can't stop in time before path end"
        double speed = Math.sqrt((vx * vx) + (vy * vy));
        double brakeDist = (speed * speed) / (2.0 * MAX_DECEL);   // physics: v^2 = 2·a·d
        terminalCost = 0;
        Pose endPose = activePath.getPath(0).getPose(1.0);
        double dxEnd = endPose.getX() - predictedX;
        double dyEnd = endPose.getY() - predictedY;
        double remainingDist = Math.sqrt((dxEnd * dxEnd) + (dyEnd * dyEnd));
        terminalTriggered = false;

        if (brakeDist > remainingDist) {
            terminalCost = WEIGHT_TERMINAL * (brakeDist - remainingDist) /* * endpointWeightMultiplier*/;
            terminalTriggered = true;

        }

        return ((WEIGHT_LOOKAHEAD * distError /* * endpointWeightMultiplier*/) + (WEIGHT_PATH * distPathError /* * endpointWeightMultiplier*/) + (WEIGHT_HEADING * headingError) + smoothnessCost + terminalCost);

    }
    private void computePurePursuit(Pose robotPose) {
        double dxField = lookaheadPose.getX() - robotPose.getX();
        double dyField = lookaheadPose.getY() - robotPose.getY();

        double heading = robotPose.getHeading();
        double dxRobot = (dxField * Math.cos(heading)) + (dyField * Math.sin(heading));
        double dyRobot = (-dxField * Math.sin(heading)) + (dyField * Math.cos(heading));

        double dist = Math.sqrt((dxRobot * dxRobot) + (dyRobot * dyRobot));
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
    private double clamp(double candidate, double min, double max) {
        if (candidate < min) {
            return min;
        }
        if (candidate > max) {
            return max;
        }
        return candidate;
    }
    private double wrapAngle(double angle) {
        while (angle > Math.PI) { //180 degrees
            angle = angle - (2 * Math.PI); //360 degrees
        }
        while (angle <= -Math.PI) { //-180 degrees
            angle = angle + (2 * Math.PI); //-360 degrees
        }
        return angle;
    }
    /**
     * Main MPC update — called once per loop.
     * Steps 2-7 will add more logic here.
     */
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
        // More logic added in later steps
    }
}