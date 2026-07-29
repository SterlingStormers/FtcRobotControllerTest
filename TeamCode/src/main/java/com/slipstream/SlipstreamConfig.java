package com.slipstream;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/21/2026
 */

@Configurable
public class SlipstreamConfig {
    public String leftFrontMotorName = "name";
    public String rightFrontMotorName = "name";
    public String leftBackMotorName = "name";
    public String rightBackMotorName = "name";

    public DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction leftBackDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction rightBackDirection = DcMotorSimple.Direction.FORWARD;

    public double maxSpeedForward = 0.0;
    public double maxSpeedStrafe = 0.0;
    public double maxTurnRate = 0.0;
    public double maxDecel = 0;

    public double vxKp = 0.05;
    public double vxKi = 0.0;
    public double vxKd = 0.0;
    public double vxKf = 1.0;

    public double vyKp = 0.05;
    public double vyKi = 0.0;
    public double vyKd = 0.0;
    public double vyKf = 1.0;

    public double omegaKp = 0.2;
    public double omegaKi = 0.0;
    public double omegaKd = 0.0;
    public double omegaKf = 1.0;
}