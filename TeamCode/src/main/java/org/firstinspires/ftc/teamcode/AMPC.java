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
    public PathChain getActivePath() { return activePath; }

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
        // More logic added in later steps
    }
}