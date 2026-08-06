package com.slipstream;

import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;

public class PathAnalyzer {
    public static class PathAnalysis {
        public double length;
        public double headingChange;
        public double maxCurvature;
        public double transitionAlignment;
    }

    public PathAnalysis analyze(Path currentPath, Path nextPath) {
        PathAnalysis result = new PathAnalysis();

        result.length = currentPath.length();
        result.headingChange = computeHeadingChange(currentPath);
        result.maxCurvature = computeMaxCurvature(currentPath);
        result.transitionAlignment = detectTransitionAlignment(currentPath, nextPath);

        return result;
    }

    private double detectTransitionAlignment(Path currentPath, Path nextPath) {
        if (nextPath == null) return 1.0;
        Vector currentEnd = currentPath.getTangentVector(1.0);
        Vector nextStart = nextPath.getTangentVector(0.0);
        double dot = currentEnd.dot(nextStart);
        double mag = currentEnd.getMagnitude() * nextStart.getMagnitude();
        if (mag < 0.001) return 1.0;
        return dot / mag;
    }

    private double computeMaxCurvature(Path path) {
        int samples = 20;
        double pathLength = path.length();
        double maxCurvature = 0;
        double step = 1.0 / (samples - 1);
        double segmentLength = step * pathLength;

        for (int i = 0; i < samples - 1; i++) {
            double t1 = i * step;
            double t2 = t1 + step;
            Vector tan1 = path.getTangentVector(t1);
            double angle1 = Math.atan2(tan1.getYComponent(), tan1.getXComponent());
            Vector tan2 = path.getTangentVector(t2);
            double angle2 = Math.atan2(tan2.getYComponent(), tan2.getXComponent());
            double rotation = Math.abs(wrapAngle(angle1 - angle2));
            double curvature = rotation / segmentLength;
            if (curvature > maxCurvature) {
                maxCurvature = curvature;
            }
        }
        return maxCurvature;
    }

    public static class Weights {
        public double progress;
        public double tangent;
        public double cross;
        public double terminal;
        public double heading;
    }

    public Weights computeWeights(PathAnalysis analysis) {
        Weights weight = new Weights();
        double curve = analysis.maxCurvature;
        weight.progress = 140 - (curve * 700);
        weight.tangent = 0.1 + (curve * 6);
        weight.cross = 3 + (curve * 30);

        double headingRate = analysis.headingChange / Math.max(1.0, analysis.length);
        weight.heading = 20 + (headingRate * 400);

        weight.terminal = 14 - (4 * analysis.transitionAlignment);

        return weight;
    }

    private double computeHeadingChange(Path path) {
        double startH = path.getPose(0.0).getHeading();
        double endH = path.getPose(1.0).getHeading();
        return Math.abs(wrapAngle(endH - startH));
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}