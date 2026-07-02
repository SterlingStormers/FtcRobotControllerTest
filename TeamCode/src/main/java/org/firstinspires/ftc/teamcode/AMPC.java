package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
@Configurable
public class AMPC {
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
    private static final double STEP_DT = 0.1;  // seconds per rollout step
    private static final int HORIZON_STEPS = 5;  // total horizon = STEP_DT * HORIZON_STEPS = 0.5s
    private static final int GRID_HALF = 1;  // 3×3×3 = 27 candidates
    private static final double GRID_STEP_FRACTION = 0.2;

    // Cost weights  applied per-step, accumulated over rollout
    public static double WEIGHT_PROGRESS = 80; //try 80
    // Cost weights — applied per-step, accumulated over rollout
    private static final double WEIGHT_PATH = 1;
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
    public static double MAX_DECEL = 244; //210
    public static double WEIGHT_TERMINAL = 10; //100
    public boolean terminalTriggered = false;
    private static final double PATH_END_TOLERANCE = 1.5;

    // Path length cache (computed once per setActivePath)
    private double pathLengthInches = 1.0;

    public boolean isPathComplete() {
        if (activePath == null) return true;
        if (currentT >= 0.95) return true;
        Pose robotPose = follower.getPose();
        Pose end = activePath.getPath(0).getPose(1.0); // will need to be changed from 0 for people with multiple paths in 1 path chain
        double dx = robotPose.getX() - end.getX();
        double dy = robotPose.getY() - end.getY();
        return Math.sqrt(dx * dx + dy * dy) < PATH_END_TOLERANCE;

    }
    public AMPC(Follower follower) {
        this.follower = follower;
    }
    public void setActivePath(PathChain path) {
        activePath = path;
        currentT = 0;
        firstLoop = true;
        pathLengthInches = Math.max(1.0, activePath.length());
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
                    double candVx = clamp(lastBestVx + (i * vxStep), -maxSpeedForward, maxSpeedForward);
                    double candVy = clamp(lastBestVy + (j * vyStep), -maxSpeedStrafe, maxSpeedStrafe);
                    double candOmega = clamp(lastBestOmega + (k * omegaStep), -maxTurnRateRad, maxTurnRateRad);

                    double cost = evaluateCandidates(candVx, candVy, candOmega, robotPose);
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
        // Zero velocity escape
        double zeroCost = evaluateCandidates(0, 0, 0, robotPose);
        if (zeroCost < bestCost) {
            bestCost = zeroCost;
            bestVx = 0;
            bestVy = 0;
            bestOmega = 0;
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
        double predictedX = startPose.getX();
        double predictedY = startPose.getY();
        double predictedHeading = startPose.getHeading();
        double predictedT = currentT;

        double speed = Math.sqrt((vx * vx) + (vy * vy));
        double tAdvancePerStep = (speed * STEP_DT) / pathLengthInches;
        double brakeDist = (speed * speed) / (2.0 * MAX_DECEL);

        double totalCost = 0;
        terminalTriggered = false;

        Pose endPose = activePath.getPath(0).getPose(1.0);

        for (int step = 1; step <= HORIZON_STEPS; step++) {
            // Forward simulate (constant velocity)
            double cosH = Math.cos(predictedHeading);
            double sinH = Math.sin(predictedHeading);
            double fieldVx = (vx * cosH) - (vy * sinH);
            double fieldVy = (vx * sinH) + (vy * cosH);
            predictedX = predictedX + (fieldVx * STEP_DT);
            predictedY = predictedY + (fieldVy * STEP_DT);
            predictedHeading = predictedHeading + (omega * STEP_DT);
            predictedT = Math.min(1.0, predictedT + tAdvancePerStep);

            // Path cost: predicted position vs path point at advanced t
            Pose pathPointAtT = activePath.getPath(0).getPose(predictedT);
            double dxPath = pathPointAtT.getX() - predictedX;
            double dyPath = pathPointAtT.getY() - predictedY;
            double distPath = Math.sqrt((dxPath * dxPath) + (dyPath * dyPath));

            // Heading cost
            double headingError = Math.abs(wrapAngle(pathPointAtT.getHeading() - predictedHeading));

            double progressPenalty = WEIGHT_PROGRESS * (1-predictedT);

            // Per-step terminal cost (guarded)
            double dxToEnd = endPose.getX() - predictedX;
            double dyToEnd = endPose.getY() - predictedY;
            double distToEnd = Math.sqrt((dxToEnd * dxToEnd) + (dyToEnd * dyToEnd));

            double stepTerminalCost = 0;
            if (brakeDist > distToEnd) {
                stepTerminalCost = WEIGHT_TERMINAL * (brakeDist - distToEnd);
                terminalTriggered = true;
            }

            totalCost = totalCost  + (WEIGHT_PATH * distPath) + (WEIGHT_HEADING * headingError) + stepTerminalCost + progressPenalty;
        }

        return totalCost;
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
            pursuitVx = (dxRobot / dist) * maxSpeedForward; // Normalize robot-to-target vector to get direction only (unit vector) then it scales by max speed
            pursuitVy = (dyRobot / dist) * maxSpeedForward; // should not be strafe cuz the robot cannot move equally at the same speed
            // TODO V3: pursuitVy should scale by maxSpeedStrafe via ellipse normalization
            // when lookahead has significant lateral component. Doesn't matter for V2
            // straight paths where lookahead is mostly along robot's forward axis.
//            double unitVectorX = dxRobot / dist;
//            double unitVectorY = dyRobot / dist;
//
//            double scale = 1.0 / Math.sqrt(((unitVectorX / maxSpeedForward) * (unitVectorX / maxSpeedForward)) + ((unitVectorY / maxSpeedStrafe) * (unitVectorY / maxSpeedStrafe)));
//
//            pursuitVx = unitVectorX * scale;
//            pursuitVy = unitVectorY * scale;
        }

        double headingError = wrapAngle(lookaheadPose.getHeading() - heading);
        pursuitOmega = clamp(headingError * HEADING_GAIN, -maxTurnRateRad, maxTurnRateRad);
    }
    private double clamp(double v, double min, double max) {
        if (v < min) return min;
        if (v > max) return max;
        return v;
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
