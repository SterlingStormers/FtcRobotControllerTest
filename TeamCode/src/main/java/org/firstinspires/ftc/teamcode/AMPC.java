package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

public class AMPC {
    private final Follower follower;

    // Robot model parameters (will be refined by SysID later)
    public double maxSpeedForward = 61.2;
    public double maxSpeedStrafe = 45.5;
    public double maxTurnRateRad = 4.0;

    // Current desired velocity output (read by VelocityController)
    public double desiredVx = 0;
    public double desiredVy = 0;
    public double desiredOmega = 0;

    // Path state
    private PathChain activePath = null;
    public double currentT = 0;

    // Search resolution for finding closest t (two-pass refinement)
    private static final int T_COARSE_STEPS = 50;
    private static final int T_FINE_STEPS = 40;
    private static final double FINE_WINDOW = 0.04;
    private static final double HEADING_GAIN = 2;

    public PathChain getActivePath() { return activePath; }

    // Lookahead configuration
    private static final double LOOKAHEAD_T_DELTA = 0.1;
    public double lookaheadT = 0;

    // MPC configuration
    private static final double HORIZON_SECONDS = 0.1;
    private static final int GRID_HALF = 1;
    private static final double GRID_STEP_FRACTION = 0.2;

    // Cost weights
    private static final double WEIGHT_LOOKAHEAD = 1.0;
    private static final double WEIGHT_PATH = 0.5;
    private static final double WEIGHT_HEADING = 10.0;

    // State carried across loops
    private double lastBestVx = 0;
    private double lastBestVy = 0;
    private double lastBestOmega = 0;
    public double lastBestCost = 0;
    public Pose lookaheadPose = new Pose(0, 0, 0);
    public double pursuitVx = 0;
    public double pursuitVy = 0;
    public double pursuitOmega = 0;

    // Smoothness weights
    private static final double WEIGHT_SMOOTHNESS_VX = 0.0;
    private static final double WEIGHT_SMOOTHNESS_VY = 0.0;
    private static final double WEIGHT_SMOOTHNESS_OMEGA = 0.0;

    private boolean firstLoop = true;
    private static final double MAX_DECEL = 183;
    private static final double WEIGHT_TERMINAL = 100;
    public double terminalCost = 0;
    public boolean terminalTriggered = false;
    private static final double SCRUBBING_FACTOR = 0.15;
    private static final double PATH_END_TOLERANCE = 1.5;

    public boolean isPathComplete() {
        if (activePath == null) return true;
        if (currentT < 0.95) return false;

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

            for (int i = 0; i <= T_COARSE_STEPS; i++) {
                double t = (double) i / T_COARSE_STEPS;
                Pose samplePose = activePath.getPath(0).getPose(t);
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

            double bestCost = Double.MAX_VALUE;
            double bestVx = lastBestVx;
            double bestVy = lastBestVy;
            double bestOmega = lastBestOmega;

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

            // Escape candidate 1: pure pursuit
            computePurePursuit(robotPose);
            double ppCost = evaluateCandidate(pursuitVx, pursuitVy, pursuitOmega, robotPose);
            if (ppCost < bestCost) {
                bestCost = ppCost;
                bestVx = pursuitVx;
                bestVy = pursuitVy;
                bestOmega = pursuitOmega;
            }

            // Escape candidate 2: brake
            double brakeCost = evaluateCandidate(0, 0, 0, robotPose);
            if (brakeCost < bestCost) {
                bestCost = brakeCost;
                bestVx = 0;
                bestVy = 0;
                bestOmega = 0;
            }

            // Commit translation from MPC optimization
            desiredVx = bestVx;
            desiredVy = bestVy;

            // will be replaced with multi step horizon
            // Always run heading P-controller against lookahead target.
            // Prevents dead-spot freezes when MPC's translation optimization picks (0,0,0)
            // but the robot still needs to rotate to match path heading.
            double targetHeading = lookaheadPose.getHeading();
            double headingError = wrapAngle(targetHeading - robotPose.getHeading());
            desiredOmega = clamp(HEADING_GAIN * headingError, -maxTurnRateRad, maxTurnRateRad);

            // Store MPC's chosen omega for warm-start consistency, but don't apply it
            lastBestVx = bestVx;
            lastBestVy = bestVy;
            lastBestOmega = desiredOmega;   // use the actual omega for warm start
            lastBestCost = bestCost;
        } else {
            desiredVx = 0;
            desiredVy = 0;
            desiredOmega = 0;
        }
    }

    private double evaluateCandidate(double vx, double vy, double omega, Pose robotPose) {
        double heading = robotPose.getHeading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double fieldVx = (vx * cosH) - (vy * sinH);
        double fieldVy = (vx * sinH) + (vy * cosH);

        double predictedX = robotPose.getX() + (fieldVx * HORIZON_SECONDS);
        double predictedY = robotPose.getY() + (fieldVy * HORIZON_SECONDS);
        double predictedHeading = heading + (omega * HORIZON_SECONDS);

        double dxError = lookaheadPose.getX() - predictedX;
        double dyError = lookaheadPose.getY() - predictedY;
        double distError = Math.sqrt((dxError * dxError) + (dyError * dyError));

        Pose closestPath = activePath.getPath(0).getPose(currentT);
        double dxPathError = closestPath.getX() - predictedX;
        double dyPathError = closestPath.getY() - predictedY;
        double distPathError = Math.sqrt((dxPathError * dxPathError) + (dyPathError * dyPathError));

        double headingError = Math.abs(wrapAngle(lookaheadPose.getHeading() - predictedHeading));

        double dVx = vx - lastBestVx;
        double dVy = vy - lastBestVy;
        double dOmega = omega - lastBestOmega;
        double smoothnessCost = (WEIGHT_SMOOTHNESS_VX * (dVx * dVx)) + (WEIGHT_SMOOTHNESS_VY * (dVy * dVy)) + (WEIGHT_SMOOTHNESS_OMEGA * (dOmega * dOmega));

        double speed = Math.sqrt((vx * vx) + (vy * vy));
        double brakeDist = (speed * speed) / (2.0 * MAX_DECEL);
        terminalCost = 0;
        Pose endPose = activePath.getPath(0).getPose(1.0);
        double dxEnd = endPose.getX() - predictedX;
        double dyEnd = endPose.getY() - predictedY;
        double remainingDist = Math.sqrt((dxEnd * dxEnd) + (dyEnd * dyEnd));
        terminalTriggered = false;

        if (brakeDist > remainingDist) {
            terminalCost = WEIGHT_TERMINAL * (brakeDist - remainingDist);
            terminalTriggered = true;
        }

        return ((WEIGHT_LOOKAHEAD * distError) + (WEIGHT_PATH * distPathError) + (WEIGHT_HEADING * headingError) + smoothnessCost + terminalCost);
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
        if (candidate < min) return min;
        if (candidate > max) return max;
        return candidate;
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) {
            angle = angle - (2 * Math.PI);
        }
        while (angle <= -Math.PI) {
            angle = angle + (2 * Math.PI);
        }
        return angle;
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