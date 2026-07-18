package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 2.0, 7/17/2026
 */

public class VelocityControllerV2 {
    private final Follower follower;
    private final AMPC ampc;
    private double integralVx = 0, lastErrorVx = 0;
    private double integralVy = 0, lastErrorVy = 0;
    private double integralOmega = 0, lastErrorOmega = 0;
    private long lastTimeNs = 0;
    private double desiredVx;
    public double actualVx;
    private double desiredVy;
    public double actualVy;
    private double desiredOmega;
    public double actualOmega;
    private double dt;
    public double effortVx;
    public double effortVy;
    public double effortOmega;

    public VelocityControllerV2(Follower follower, AMPC ampc) {
        this.follower = follower;
        this.ampc = ampc;
    }

    public void velocity() {
        desiredVx = ampc.desiredVx;
        desiredVy = ampc.desiredVy;
        desiredOmega = ampc.desiredOmega;

        double fieldVelX = follower.getVelocity().getXComponent();
        double fieldVelY = follower.getVelocity().getYComponent();
        double heading = follower.getPose().getHeading();

        actualVx = fieldVelX * Math.cos(heading) + fieldVelY * Math.sin(heading);
        actualVy = -fieldVelX * Math.sin(heading) + fieldVelY * Math.cos(heading);
        actualOmega = follower.getAngularVelocity();

        long now = System.nanoTime();
        dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        effortVx = pidfComputeVx();
        effortVy = pidfComputeVy();
        effortOmega = pidfComputeOmega();
    }

    private double pidfComputeVx() {
        double errorVx = desiredVx - actualVx;
        integralVx += errorVx * dt;
        double derivativeVx = (errorVx - lastErrorVx) / dt;
        lastErrorVx = errorVx;
        return SlipstreamConstants.vxKf * desiredVx + SlipstreamConstants.vxKp * errorVx + SlipstreamConstants.vxKi * integralVx + SlipstreamConstants.vxKd * derivativeVx;
    }

    private double pidfComputeVy() {
        double errorVy = desiredVy - actualVy;
        integralVy += errorVy * dt;
        double derivativeVy = (errorVy - lastErrorVy) / dt;
        lastErrorVy = errorVy;
        return SlipstreamConstants.vyKf * desiredVy + SlipstreamConstants.vyKp * errorVy + SlipstreamConstants.vyKi * integralVy + SlipstreamConstants.vyKd * derivativeVy;
    }

    private double pidfComputeOmega() {
        double errorOmega = desiredOmega - actualOmega;
        integralOmega += errorOmega * dt;
        double derivativeOmega = (errorOmega - lastErrorOmega) / dt;
        lastErrorOmega = errorOmega;
        return SlipstreamConstants.omegaKf * desiredOmega + SlipstreamConstants.omegaKp * errorOmega + SlipstreamConstants.omegaKi * integralOmega + SlipstreamConstants.omegaKd * derivativeOmega;
    }

    public void reset() {
        integralVx = 0;
        lastErrorVx = 0;
        integralVy = 0;
        lastErrorVy = 0;
        integralOmega = 0;
        lastErrorOmega = 0;
        lastTimeNs = 0;
    }
}

