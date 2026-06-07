package org.firstinspires.ftc.teamcode;

public class MPC {
    // Robot model parameters  owned by MPC, read by Kinematics and others
    // Will be refined by SysID logic (added later)
    private double maxSpeedForward = 60.0;   // placeholder, in/s
    private double maxSpeedStrafe = 50.0;    // placeholder, in/s
    private double maxTurnRate = 4.0;        // placeholder, rad/s

    // Getters for other layers to read the model
    public double getMaxSpeedForward() {
        return maxSpeedForward;
    }
    public double getMaxSpeedStrafe() {
        return maxSpeedStrafe;
    }
    public double getMaxTurnRate() {
        return maxTurnRate;
    }
    // Real planning logic and SysID will be added later.
    // For now this class just holds the model parameters.
}