package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;

public class VelocityController {
    private final Follower follower;
    private final MecanumKinematics kinematics;
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
    public VelocityController(Follower follower, MecanumKinematics kinematics) {
        this.follower = follower;
        this.kinematics = kinematics;
    }
    public void track(double desiredVx, double desiredVy, double desiredOmega) {
        // Read actual velocities in robot frame
        double fieldVelX = follower.getVelocity().getXComponent();
        double fieldVelY = follower.getVelocity().getYComponent();
        double heading = follower.getPose().getHeading();
        double actualVx = fieldVelX * Math.cos(heading) + fieldVelY * Math.sin(heading);
        double actualVy = -fieldVelX * Math.sin(heading) + fieldVelY * Math.cos(heading);
        double actualOmega = follower.getAngularVelocity();
        // Compute dt
        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        // PIDF computations
        double effortVx = pidfCompute(desiredVx, actualVx, KFF_FORWARD, KP_FORWARD, KI_FORWARD, KD_FORWARD, dt,"vx");
        double effortVy = pidfCompute(desiredVy, actualVy, KFF_STRAFE, KP_STRAFE, KI_STRAFE, KD_STRAFE, dt,"vy");
        double effortOmega = pidfCompute(desiredOmega, actualOmega, KFF_OMEGA, KP_OMEGA, KI_OMEGA, KD_OMEGA, dt,"omega");
        // Pass to Kinematics
        kinematics.drive(effortVx, effortVy, effortOmega);
    }
    private double pidfCompute(double desired, double actual, double kff, double kp, double ki, double kd, double dt, String axis) {
        double error = desired - actual;
        // Per axis
        double integral, lastError;
        switch (axis) {
            case "vx": integral = integralVx; lastError = lastErrorVx; break;
            case "vy": integral = integralVy; lastError = lastErrorVy; break;
            case "omega": integral = integralOmega; lastError = lastErrorOmega; break;
            default: throw new IllegalArgumentException("Unknown axis: " + axis);
        }
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        // Save
        switch (axis) {
            case "vx": integralVx = integral; lastErrorVx = error; break;
            case "vy": integralVy = integral; lastErrorVy = error; break;
            case "omega": integralOmega = integral; lastErrorOmega = error; break;
        }
        return kff * desired + kp * error + ki * integral + kd * derivative;
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