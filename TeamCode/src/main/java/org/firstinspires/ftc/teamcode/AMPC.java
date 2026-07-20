package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 4.0, 7/17/2026
 */

@Configurable   //remove before release
public class AMPC {  // Version 1.4.0
    private final Follower follower;
    private final double baseMaxSpeedForward;
    private final double baseMaxSpeedStrafe;
    private final double baseMaxTurnRate;
    private PathChain activePath = null;
    public PathChain getActivePath() {
        return activePath;
    }
    private double lastBestVx = 0;
    private double lastBestVy = 0;
    private double lastBestOmega = 0;
    private double pursuitVx = 0;
    private double pursuitVy = 0;
    private double pursuitOmega = 0;
    private boolean firstLoop = true;
    private double pathLengthInches = 1.0;
    private static final int T_COARSE_STEPS = 50;
    private static final int T_FINE_STEPS = 40;
    private static final double FINE_WINDOW = 0.04;
    private static final double HEADING_GAIN = 2;
    private static final double LOOKAHEAD_T_DELTA = 0.1;
    private static final double STEP_DT = 0.1;
    private static final int HORIZON_STEPS = 5;
    private static final int GRID_HALF = 1;
    private static final double GRID_STEP_FRACTION = 0.2;
    private static final double WEIGHT_PROGRESS = 80;
    private static final double WEIGHT_CROSS = 3.0;
    private static final double WEIGHT_ALONG = 1.0;
    private static final double WEIGHT_TANGENT = 0.1;
    private static final double WEIGHT_HEADING = 10.0;
    private static final double WEIGHT_TERMINAL = 10;
    private static final double PATH_END_TOLERANCE = 1.5;
    private static final double LEARNING_RATE = 0.05;
    public double maxSpeedForward;
    public double maxSpeedStrafe;
    public double maxTurnRateRad;

    public double desiredVx = 0;

    public double desiredVy = 0;

    public double desiredOmega = 0;

    public double currentT = 0;
    public double lookaheadT = 0;
    public Pose lookaheadPose = new Pose(0, 0, 0);
    public double lastBestCost = 0;
    public boolean terminalTriggered = false;
    public double lastCommandedVx = 0;
    public double lastCommandedVy = 0;
    public double lastCommandedOmega = 0;
    public double actualVx = 0;
    public double actualVy = 0;
    public double actualOmega = 0;
    public double sysIDRatioVx = 1.0;
    public double sysIDRatioVy = 1.0;
    public double sysIDRatioOmega = 1.0;
    public double filteredRatioVx = 1.0;
    public double filteredRatioVy = 1.0;
    public double filteredRatioOmega = 1.0;

    public AMPC(Follower follower) {
        this.follower = follower;
        this.baseMaxSpeedForward = SlipstreamConstants.maxSpeedForward;
        this.baseMaxSpeedStrafe = SlipstreamConstants.maxSpeedStrafe;
        this.baseMaxTurnRate = SlipstreamConstants.maxTurnRate;
        this.maxSpeedForward = baseMaxSpeedForward;
        this.maxSpeedStrafe = baseMaxSpeedStrafe;
        this.maxTurnRateRad = baseMaxTurnRate;
    }
    public void observeSysID() {
        lastCommandedVx = desiredVx;
        lastCommandedVy = desiredVy;
        lastCommandedOmega = desiredOmega;

        double heading = follower.getPose().getHeading();
        double fieldVx = follower.getVelocity().getXComponent();
        double fieldVy = follower.getVelocity().getYComponent();
        actualVx = fieldVx * Math.cos(heading) + fieldVy * Math.sin(heading);
        actualVy = -fieldVx * Math.sin(heading) + fieldVy * Math.cos(heading);
        actualOmega = follower.getAngularVelocity();

        if ((Math.abs(desiredVx) > 15) && (Math.abs(actualVx) > 15)) {
            sysIDRatioVx = actualVx / desiredVx;
            if (sysIDRatioVx > 0 && sysIDRatioVx < 2.0) {
                filteredRatioVx = filteredRatioVx * (1 - LEARNING_RATE) + (sysIDRatioVx * LEARNING_RATE);
            }
        }
        if ((Math.abs(desiredVy) > 10) && (Math.abs(actualVy) > 10)) {
            sysIDRatioVy = actualVy / desiredVy;
            if (sysIDRatioVy > 0 && sysIDRatioVy < 2.0) {
                filteredRatioVy = filteredRatioVy * (1 - LEARNING_RATE) + (sysIDRatioVy * LEARNING_RATE);
            }
        }
        if ((Math.abs(desiredOmega) > 0.5) && (Math.abs(actualOmega) > 0.5)) {
            sysIDRatioOmega = actualOmega / desiredOmega;
            if (sysIDRatioOmega > 0 && sysIDRatioOmega < 2.0) {
                filteredRatioOmega = filteredRatioOmega * (1 - LEARNING_RATE) + (sysIDRatioOmega * LEARNING_RATE);
            }
        }

        // Adaptation  overwrite max speeds with filtered ratios
        // Clamped so bad ratios can't send max speeds to zero or infinity
        double clampedVx = Math.max(0.7, Math.min(1.3, filteredRatioVx));
        double clampedVy = Math.max(0.7, Math.min(1.3, filteredRatioVy));
        double clampedOmega = Math.max(0.7, Math.min(1.3, filteredRatioOmega));

        maxSpeedForward = baseMaxSpeedForward * clampedVx;
        maxSpeedStrafe = baseMaxSpeedStrafe * clampedVy;
        maxTurnRateRad = baseMaxTurnRate * clampedOmega;
    }

    public boolean isPathComplete() {
        if (activePath == null) return true;
        if (currentT >= 0.95) return true;
        Pose robotPose = follower.getPose();
        Pose end = activePath.getPath(0).getPose(1.0);
        double dx = robotPose.getX() - end.getX();
        double dy = robotPose.getY() - end.getY();
        return Math.sqrt(dx * dx + dy * dy) < PATH_END_TOLERANCE;
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
        double tAdvancePerStep = (speed * STEP_DT) / pathLengthInches;
        double brakeDist = (speed * speed) / (2.0 * SlipstreamConstants.maxDecel);

        double totalCost = 0;
        terminalTriggered = false;

        Pose endPose = activePath.getPath(0).getPose(1.0);

        for (int step = 1; step <= HORIZON_STEPS; step++) {
            // Forward simulate (constant velocity)
            double cosH = Math.cos(predictedHeading);
            double sinH = Math.sin(predictedHeading);
            double fieldVx = (vx * cosH) - (vy * sinH);
            double fieldVy = (vx * sinH) + (vy * cosH);
            predictedX += fieldVx * STEP_DT;
            predictedY += fieldVy * STEP_DT;
            predictedHeading += omega * STEP_DT;
            predictedT = Math.min(1.0, predictedT + tAdvancePerStep);

            // Path point + tangent at advanced t
            Pose pathPointAtT = activePath.getPath(0).getPose(predictedT);
            Vector tangent = activePath.getPath(0).getTangentVector(predictedT);
            double tanMag = tangent.getMagnitude();
            double tX = tanMag > 0.001 ? tangent.getXComponent() / tanMag : 1.0;
            double tY = tanMag > 0.001 ? tangent.getYComponent() / tanMag : 0.0;

            // Decompose position error
            double dx = predictedX - pathPointAtT.getX();
            double dy = predictedY - pathPointAtT.getY();
            double crossTrack = Math.abs(-dx * tY + dy * tX);
            double alongTrack = Math.abs(dx * tX + dy * tY);

            // Heading error
            double headingError = Math.abs(wrapAngle(pathPointAtT.getHeading() - predictedHeading));

            // Progress reward
            double progressPenalty = WEIGHT_PROGRESS * (1 - predictedT);

            //  Tangent-alignment cost: velocity direction should match path tangent
            double vMag = Math.sqrt(fieldVx * fieldVx + fieldVy * fieldVy);
            double tangentAlignmentCost = 0;
            if (vMag > 0.1) {
                double vUnitX = fieldVx / vMag;
                double vUnitY = fieldVy / vMag;
                double alignmentError = 1.0 - (vUnitX * tX + vUnitY * tY);
                tangentAlignmentCost = WEIGHT_TANGENT * alignmentError * vMag;
            }

            // Terminal cost (brake before overshoot)
            // Hybrid: use arc length remaining when far from end, Euclidean when close
            double euclidToEnd = Math.sqrt((endPose.getX() - predictedX) * (endPose.getX() - predictedX) + (endPose.getY() - predictedY) * (endPose.getY() - predictedY));
            double arcToEnd = pathLengthInches * (1.0 - predictedT);
            double distToEnd;
            if (predictedT < 0.85) {
                distToEnd = euclidToEnd;  // aggressive far away
            } else if (predictedT < 0.95) {
                distToEnd = arcToEnd; // accurate close
            } else {
                distToEnd = euclidToEnd; // switch back near endpoint to avoid stall
            }
            double stepTerminalCost = 0;
            if (brakeDist > distToEnd) {
                stepTerminalCost = WEIGHT_TERMINAL * (brakeDist - distToEnd);
                terminalTriggered = true;
            }

            totalCost += (WEIGHT_CROSS * crossTrack) + (WEIGHT_ALONG * alongTrack) + (WEIGHT_HEADING * headingError) + progressPenalty + tangentAlignmentCost + stepTerminalCost;
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
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
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
        observeSysID();
    }
}