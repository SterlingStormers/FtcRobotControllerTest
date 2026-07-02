package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

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
    private static final double STEP_DT = 0.1;
    private static final int HORIZON_STEPS = 5;
    private static final int GRID_HALF = 1;
    private static final double GRID_STEP_FRACTION = 0.2;

    // Cost weights
    public static double WEIGHT_PROGRESS = 80;
    private static final double WEIGHT_PATH = 1;   // cross-track weight
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
    public static double MAX_DECEL = 244;
    public static double WEIGHT_TERMINAL = 10;
    public boolean terminalTriggered = false;
    private static final double PATH_END_TOLERANCE = 1.5;

    private double pathLengthInches = 1.0;

    public boolean isPathComplete() {
        if (activePath == null) return true;
        if (currentT >= 0.95) return true;
        Pose robotPose = follower.getPose();
        Pose end = activePath.getPath(0).getPose(1.0);
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

    /**
     * Get unit tangent vector at path parameter t.
     * Returns [tangentX, tangentY] as unit vector.
     */
    private double[] getUnitTangent(double t) {
        Vector v = activePath.getPath(0).getTangentVector(t);
        double x = v.getXComponent();
        double y = v.getYComponent();
        double mag = v.getMagnitude();
        if (mag > 0.001) {
            return new double[]{x / mag, y / mag};
        }
        return new double[]{1.0, 0.0};  // fallback if degenerate
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
                double t = tMin + ((tMax - tMin) * ((double) i / T_FINE_STEPS));
                Pose samplePose = activePath.getPath(0).getPose(t);
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
            lookaheadPose = activePath.getPath(0).getPose(lookaheadT);
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

        computePurePursuit(robotPose);
        double ppCost = evaluateCandidates(pursuitVx, pursuitVy, pursuitOmega, robotPose);
        if (ppCost < bestCost) {
            bestCost = ppCost;
            bestVx = pursuitVx;
            bestVy = pursuitVy;
            bestOmega = pursuitOmega;
        }

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
        double brakeDist = (speed * speed) / (2.0 * MAX_DECEL);

        double totalCost = 0;
        terminalTriggered = false;

        Pose endPose = activePath.getPath(0).getPose(1.0);

        for (int step = 1; step <= HORIZON_STEPS; step++) {
            // === TANGENT AT CURRENT predictedT (for projected speed) ===
            double[] tangent = getUnitTangent(predictedT);
            double tangentX = tangent[0];
            double tangentY = tangent[1];

            // === FORWARD SIMULATE ===
            double cosH = Math.cos(predictedHeading);
            double sinH = Math.sin(predictedHeading);
            double fieldVx = (vx * cosH) - (vy * sinH);
            double fieldVy = (vx * sinH) + (vy * cosH);

            // === PROJECTED SPEED FOR PATH PROGRESS ===
            // speedAlongPath = component of field velocity along path tangent
            double speedAlongPath = (fieldVx * tangentX) + (fieldVy * tangentY);
            double tAdvancePerStep = (Math.max(0, speedAlongPath) * STEP_DT) / pathLengthInches;

            predictedX = predictedX + (fieldVx * STEP_DT);
            predictedY = predictedY + (fieldVy * STEP_DT);
            predictedHeading = predictedHeading + (omega * STEP_DT);
            predictedT = Math.min(1.0, predictedT + tAdvancePerStep);

            // === CROSS-TRACK ERROR (at new predictedT) ===
            Pose pathPointAtT = activePath.getPath(0).getPose(predictedT);
            double[] newTangent = getUnitTangent(predictedT);
            double dx = predictedX - pathPointAtT.getX();
            double dy = predictedY - pathPointAtT.getY();
            double crossTrack = Math.abs((-dx * newTangent[1]) + (dy * newTangent[0]));

            // === HEADING ERROR ===
            double headingError = Math.abs(wrapAngle(pathPointAtT.getHeading() - predictedHeading));

            // === PROGRESS REWARD ===
            double progressPenalty = WEIGHT_PROGRESS * (1.0 - predictedT);

            // === TERMINAL COST ===
            double dxToEnd = endPose.getX() - predictedX;
            double dyToEnd = endPose.getY() - predictedY;
            double distToEnd = Math.sqrt((dxToEnd * dxToEnd) + (dyToEnd * dyToEnd));

            double stepTerminalCost = 0;
            if (brakeDist > distToEnd) {
                stepTerminalCost = WEIGHT_TERMINAL * (brakeDist - distToEnd);
                terminalTriggered = true;
            }

            totalCost = totalCost
                    + (WEIGHT_PATH * crossTrack)
                    + (WEIGHT_HEADING * headingError)
                    + progressPenalty
                    + stepTerminalCost;
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
            pursuitVx = (dxRobot / dist) * maxSpeedForward;
            pursuitVy = (dyRobot / dist) * maxSpeedForward;
            // TODO V3: pursuitVy should scale by maxSpeedStrafe via ellipse normalization
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