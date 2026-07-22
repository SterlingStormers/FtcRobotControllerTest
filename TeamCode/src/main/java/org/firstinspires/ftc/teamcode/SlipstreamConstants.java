package org.firstinspires.ftc.teamcode;
import com.slipstream.SlipstreamConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

public class SlipstreamConstants extends SlipstreamConfig {
    public SlipstreamConstants() {

        // Motor names
        leftFrontMotorName = "name";
        rightFrontMotorName = "name";
        leftBackMotorName = "name";
        rightBackMotorName = "name";

        /**
         * Note: Slipstream is strictly for motor mode set to FLOAT
         * This is automatically handled by the Kinematics.
         */

        // Motor directions
        leftFrontDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontDirection = DcMotorSimple.Direction.FORWARD;
        leftBackDirection = DcMotorSimple.Direction.REVERSE;
        rightBackDirection = DcMotorSimple.Direction.FORWARD;

        // Physical limits
        maxSpeedForward = 0.0;    // Maximum forward velocity in inches/second. Measure with SlipstreamMaxSpeedForwardTest.
        maxSpeedStrafe = 0.0;    // Maximum strafe velocity in inches/second. Measure with SlipstreamMaxSpeedStrafeTest.
        maxTurnRate = 0.0;    // Maximum turn velocity in radians/second. Measure with SlipstreamMaxSpeedTurnTest.
        maxDecel = 0;    // Maximum deceleration. Measure with SlipstreamBrakeDecelTest.

        /**
         * Note: These are stock PIDF values.
         * Tuning these values is optional and can be done for additional performance.
         * Visit documentation to learn how to tune PIDF values.
         */

        // Forward velocity PIDF
        vxKp = 0.05;
        vxKi = 0.0;
        vxKd = 0.0;
        vxKf = 1.0;

        // Strafe velocity PIDF
        vyKp = 0.05;
        vyKi = 0.0;
        vyKd = 0.0;
        vyKf = 1.0;

        // Angular velocity PIDF
        omegaKp = 0.2;
        omegaKi = 0.0;
        omegaKd = 0.0;
        omegaKf = 1.0;
    }
}