package com.slipstream;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class DynamicVelocityProfile {
    private static final int T_COARSE_STEPS = 30;
    private static final int T_FINE_STEPS = 20;
    private static final double FINE_WINDOW = 0.02;
    private static final double MAX_T_ADVANCE = 0.15;  // per loop
    private static final double MAX_T_RETREAT = 0.02;  // per loop
    private PathChain activePath = null;
    public double currentT = 0;
    private double pathLength = 1.0;
    private boolean firstLoop = true;
    public double desiredVx = 0;
    public double desiredVy = 0;
    public double desiredOmega = 0;
    private final Follower follower;
    private final SlipstreamConfig config;

    public DynamicVelocityProfile(Follower follower, SlipstreamConfig config) {
        this.follower = follower;
        this.config = config;
    }

    public PathChain getActivePath() {
        return activePath;
    }

    public void setActivePath(PathChain path) {
        activePath = path;
        currentT = 0;
        firstLoop = true;
        pathLength = Math.max(1.0, path.length());
    }

    public void updateClosestT() {
        if (activePath == null) return;
        Pose robotPose = follower.getPose();
        double tMin, tMax;

        if (firstLoop) {
            tMin = 0.0;
            tMax = 1.0;
            firstLoop = false;
        } else {
            tMin = Math.max(0.0, currentT - MAX_T_RETREAT);
            tMax = Math.min(1.0, currentT + MAX_T_ADVANCE);
        }

        double bestT = currentT;
        double bestDistSq = Double.MAX_VALUE;
        double coarseStep = (tMax - tMin) / T_COARSE_STEPS;
        for (int i = 0; i <= T_COARSE_STEPS; i++) {
            double t = tMin + i * coarseStep;
            Pose samplePose = activePath.getPath(0).getPose(t);
            double dx = samplePose.getX() - robotPose.getX();
            double dy = samplePose.getY() - robotPose.getY();
            double distSq = dx * dx + dy * dy;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestT = t;
            }
        }


        double fineMin = Math.max(tMin, bestT - FINE_WINDOW);
        double fineMax = Math.min(tMax, bestT + FINE_WINDOW);
        double fineStep = (fineMax - fineMin) / T_FINE_STEPS;
        for (int i = 0; i <= T_FINE_STEPS; i++) {
            double t = fineMin + i * fineStep;
            Pose samplePose = activePath.getPath(0).getPose(t);
            double dx = samplePose.getX() - robotPose.getX();
            double dy = samplePose.getY() - robotPose.getY();
            double distSq = dx * dx + dy * dy;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestT = t;
            }
        }

        currentT = bestT;
    }

    public void update() {
        if (activePath == null) {
            desiredVx = 0;
            desiredVy = 0;
            desiredOmega = 0;
            return;
        }
        updateClosestT();
        // TODO: compute speed (live backward pass + curvature + kinematic)
        // TODO: compute direction (tangent + cross-track blend)
        // TODO: assign desiredVx, desiredVy, desiredOmega
    }
}