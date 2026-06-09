package org.firstinspires.ftc.teamcode;

public class MPC {
    // Robot model parameters
    double maxSpeedForward = 54.8;
    double maxSpeedStrafe = 46.0;
    double maxTurnRateRad = 4.0;
    // Current desired velocity output — for testing, hardcoded
    // Later this gets computed by compute() based on path geometry
    double desiredVx = 0;
    double desiredVy = 0;
    double desiredOmega = 0;
    // Real compute() logic comes later — will eventually set desiredVx etc. internally
}