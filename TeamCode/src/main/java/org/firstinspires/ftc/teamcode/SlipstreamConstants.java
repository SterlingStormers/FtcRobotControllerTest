package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/17/2026
 */

/**
 * User-facing configuration for Slipstream.
 *
 * All values here are what teams need to set for their robot.
 * Ensure all constants are calibrated and tuned at max battery voltage.
 * Panels Live-Tuning Enabled
 * For additional information, visit our documentation.
 */

@Configurable
public class SlipstreamConstants {

    // Motor names
    public static String leftFrontMotorName = "front_left_drive";
    public static String rightFrontMotorName = "front_right_drive";
    public static String leftBackMotorName = "back_left_drive";
    public static String rightBackMotorName = "back_right_drive";

    /**
     * Note: Slipstream is strictly for motor mode set to FLOAT
     * This is automatically handled by the Kinematics.
     */

    // Motor directions
    public static DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftBackDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightBackDirection = DcMotorSimple.Direction.FORWARD;

    // Physical limits
    public static double maxSpeedForward = 61.2;    // Maximum forward velocity in inches/second. Measure with SlipstreamMaxSpeedForwardTest.

    public static double maxSpeedStrafe = 45.5;    // Maximum strafe velocity in inches/second. Measure with SlipstreamMaxSpeedStrafeTest.
    public static double maxTurnRate = 4.0;    // Maximum turn velocity in radians/second. Measure with SlipstreamMaxSpeedTurnTest.
    public static double maxDecel = 244;    // Maximum deceleration. Measure with SlipstreamBrakeDecelTest.

    /**
     * Note: These are stock PIDF values.
     * Tuning these values is optional and can be done for additional performance.
     * Visit documentation to learn how to tune PIDF values.
     */

    // Forward velocity PIDF
    public static double vxKp = 0.05;
    public static double vxKi = 0.0;
    public static double vxKd = 0.0;
    public static double vxKf = 1.0;

    // Strafe velocity PIDF
    public static double vyKp = 0.05;
    public static double vyKi = 0.0;
    public static double vyKd = 0.0;
    public static double vyKf = 1.0;

    // Angular velocity PIDF
    public static double omegaKp = 0.2;
    public static double omegaKi = 0.0;
    public static double omegaKd = 0.0;
    public static double omegaKf = 1.0;
}