package com.slipstream;

import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;

public class PathAnalyzer {

    public static class PathAnalysis {
        public double length;
        public double headingChange;
        public double maxCurvature;
        public boolean isDirectionReversal;
    }

    public PathAnalysis analyze(Path currentPath, Path previousPath) {
        PathAnalysis result = new PathAnalysis();

        result.length = currentPath.length();
        result.headingChange = computeHeadingChange(currentPath);
        // TODO: max curvature
        // TODO: direction reversal detection with previousPath

        return result;
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