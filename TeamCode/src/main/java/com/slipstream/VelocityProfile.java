package com.slipstream;

import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;

public class VelocityProfile {
    private static final int SAMPLES = 100;
    private static final double FRICTION = 0.7;
    private static final double GRAVITY = 386.0;
    private double[] velocities = new double[SAMPLES + 1];
    private double pathLength = 0;

    public void compute(Path path, double startVel, double endVel, double maxSpeedFwd, double maxSpeedStr, double maxAccel, double maxDecel) {
        double[] vLimit = new double[SAMPLES + 1];
        for (int i = 0; i <= SAMPLES; i++) {
            double t = (double) i / SAMPLES;
            double K = computeCurvature(path, t);
            double vCurve = K > 0.001 ? Math.sqrt(FRICTION * GRAVITY / K) : Double.MAX_VALUE;
            //ask why we need forward pass
        }

        // 1. Store pathLength, compute ds
        // 2. Loop 1: fill vLimit[] with min(curvature limit, maxSpeed)
        // 3. Loop 2: forward pass — fill vForward[]
        // 4. Loop 3: backward pass — fill vBackward[]
        // 5. Combine: velocities[i] = min(vForward[i], vBackward[i])
    }

    private double computeCurvature(Path path, double t) {
        double t1 = Math.max(0, t - 0.01);
        double t2 = Math.min(1, t + 0.01);

        Vector tan1 = path.getTangentVector(t1);
        Vector tan2 = path.getTangentVector(t2);

        double angle1 = Math.atan2(tan1.getYComponent(), tan1.getXComponent());
        double angle2 = Math.atan2(tan2.getYComponent(), tan2.getXComponent());

        double rotation = Math.abs(wrapAngle(angle2 - angle1));

        Pose p1 = path.getPose(t1);
        Pose p2 = path.getPose(t2);
        double segmentLength = Math.hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());

        return segmentLength > 0.001 ? rotation / segmentLength : 0;
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}