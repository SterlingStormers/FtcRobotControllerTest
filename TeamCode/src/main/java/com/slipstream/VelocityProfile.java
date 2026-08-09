package com.slipstream;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;

public class VelocityProfile {
    private static final int SAMPLES = 100;
    private static final double FRICTION = 0.7;
    private double[] velocities = new double[SAMPLES + 1];
    private double pathLength = 0;

    public void compute(Path path, double endVel, double maxSpeedFwd, double maxSpeedStr, double maxDecel) {
        pathLength = path.length();
        double segmentLength = pathLength / SAMPLES;

        double[] vLimit = new double[SAMPLES + 1];
        for (int i = 0; i <= SAMPLES; i++) {
            double t = (double) i / SAMPLES;

            double K = computeCurvature(path, t);
            double vCurve = K > 0.001 ? Math.sqrt(FRICTION * 386 / K) : Double.MAX_VALUE;

            Vector tan = path.getTangentVector(t);
            double tMag = tan.getMagnitude();
            double tX = tMag > 0.001 ? tan.getXComponent() / tMag : 1.0;
            double tY = tMag > 0.001 ? tan.getYComponent() / tMag : 0.0;
            double heading = path.getPose(t).getHeading();
            double robotVx = tX * Math.cos(heading) + tY * Math.sin(heading);
            double robotVy = -tX * Math.sin(heading) + tY * Math.cos(heading);
            double vFwdLim = maxSpeedFwd / Math.max(0.001, Math.abs(robotVx));
            double vStrLim = maxSpeedStr / Math.max(0.001, Math.abs(robotVy));
            double vKinematic = Math.min(vFwdLim, vStrLim);

            vLimit[i] = Math.min(vCurve, vKinematic);
        }

        velocities[SAMPLES] = endVel;
        for (int l = SAMPLES - 1; l >= 0; l--) {
            double vDecel = Math.sqrt(velocities[l+1] * velocities[l+1] + 2 * maxDecel * segmentLength);
            velocities[l] = Math.min(vDecel, vLimit[l]);
        }
    }

    public double getMaxSpeedAt(double t) {
        int sample = Math.max(0, Math.min(SAMPLES, (int) (t * SAMPLES)));
        return velocities[sample];
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