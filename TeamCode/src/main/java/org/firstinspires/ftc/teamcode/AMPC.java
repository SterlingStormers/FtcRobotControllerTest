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
    private static final double FINE_WINDOW = 0.04;   // ±window around coarse best
    private static final double HEADING_GAIN = 2;
    public PathChain getActivePath() { return activePath; }
    // Lookahead configuration
    private static final double LOOKAHEAD_T_DELTA = 0.1;   // ~5 in on 50 in test path
    // Lookahead output (read by Step 3+)
    public double lookaheadT = 0;

    // MPC configuration
    private static final double HORIZON_SECONDS = 0.1;
    private static final int GRID_HALF = 1; // 1 → 3×3×3 = 27 candidates
    private static final double GRID_STEP_FRACTION = 0.2; // ±20% of max per step

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
    public AMPC(Follower follower) {
        this.follower = follower;
    }
    public void setActivePath(PathChain path) {
        this.activePath = path;
        this.currentT = 0;
    }
    // may need removal (code below)=====+
    public boolean hasActivePath() {
        return activePath != null;
    }
// may need removal (code above)======

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

            double vxStep = GRID_STEP_FRACTION * maxSpeedForward;
            double vyStep = GRID_STEP_FRACTION * maxSpeedStrafe;
            double omegaStep = GRID_STEP_FRACTION * maxTurnRateRad;

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
        double fieldVx = (vx * cosH) - (vy * sinH);
        double fieldVy = (vx * sinH) + (vy * cosH);

        // Euler integration over horizon
        double predictedX = robotPose.getX() + (fieldVx * HORIZON_SECONDS);
        double predictedY = robotPose.getY() + (fieldVy * HORIZON_SECONDS);
        double predictedHeading = heading + (omega * HORIZON_SECONDS);
        // predicted heading not used in Step 4 cost, but available for Step 5+

        // Cost term 1: distance from predicted pose to lookahead
        double dxError = lookaheadPose.getX() - predictedX;
        double dyError = lookaheadPose.getY() - predictedY;
        double distError = Math.sqrt((dxError * dxError) + (dyError * dyError));
        // Cost term 2: distance from predicted pose to closest point on path
        Pose closestPath = activePath.getPath(0).getPose(currentT);
        double dxPathError = closestPath.getX() - predictedX;
        double dyPathError = closestPath.getY() - predictedY;
        double distPathError = Math.sqrt((dxPathError * dxPathError) + (dyPathError * dyPathError));
        double headingError = Math.abs(wrapAngle(lookaheadPose.getHeading() - predictedHeading));

        return (WEIGHT_LOOKAHEAD * distError) + (WEIGHT_PATH * distPathError) + (WEIGHT_HEADING * headingError);

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