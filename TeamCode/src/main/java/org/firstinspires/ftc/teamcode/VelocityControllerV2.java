package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;

public class VelocityControllerV2 {
    private final Follower follower;
    private final MPC mpc;
    // PIDF gain placeholders, we'll tune these
    private static final double KFF_FORWARD = 1.0;   // feedforward: command = desired velocity
    private static final double KP_FORWARD = 0.05;
    private static final double KI_FORWARD = 0.0;
    private static final double KD_FORWARD = 0.0;
    private static final double KFF_STRAFE = 1.0;
    private static final double KP_STRAFE = 0.05;
    private static final double KI_STRAFE = 0.0;
    private static final double KD_STRAFE = 0.0;
    private static final double KFF_OMEGA = 1.0;
    private static final double KP_OMEGA = 0.2;
    private static final double KI_OMEGA = 0.0;
    private static final double KD_OMEGA = 0.0;
    // PIDF state for each axis
    private double integralVx = 0, lastErrorVx = 0;
    private double integralVy = 0, lastErrorVy = 0;
    private double integralOmega = 0, lastErrorOmega = 0;
    private long lastTimeNs = 0;
    private double desiredVx;
    private double actualVx;
    private double desiredVy;
    private double actualVy;
    private double desiredOmega;
    private double actualOmega;
    private double dt;
    public double effortVx;
    public double effortVy;
    public double effortOmega;

    public VelocityControllerV2(Follower follower, MPC mpc) {
        this.follower = follower;
        this.mpc = mpc;
    }
    public void velocity(){
        desiredVx = mpc.desiredVx;
        desiredVy = mpc.desiredVy;
        desiredOmega = mpc.desiredOmega;
        double fieldVelX = follower.getVelocity().getXComponent();
        double fieldVelY = follower.getVelocity().getYComponent();
        double heading = follower.getPose().getHeading();
        actualVx = fieldVelX * Math.cos(heading) + fieldVelY * Math.sin(heading);
        actualVy = -fieldVelX * Math.sin(heading) + fieldVelY * Math.cos(heading);
        actualOmega = follower.getAngularVelocity();
        // Compute dt
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
        return KFF_FORWARD * desiredVx + KP_FORWARD * errorVx + KI_FORWARD * integralVx + KD_FORWARD * derivativeVx;
    }
    private double pidfComputeVy() {
        double errorVy = desiredVy - actualVy;
        integralVy += errorVy * dt;
        double derivativeVy = (errorVy - lastErrorVy) / dt;
        lastErrorVy = errorVy;
        return KFF_STRAFE * desiredVy + KP_STRAFE * errorVy + KI_STRAFE * integralVy + KD_STRAFE * derivativeVy;
    }
    private double pidfComputeOmega() {
        double errorOmega = desiredOmega - actualOmega;
        integralOmega += errorOmega * dt;
        double derivativeOmega = (errorOmega - lastErrorOmega) / dt;
        lastErrorOmega = errorOmega;
        return KFF_OMEGA * desiredOmega + KP_OMEGA * errorOmega + KI_OMEGA * integralOmega + KD_OMEGA * derivativeOmega;
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
