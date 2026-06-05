package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LightweightMPC {
    private final Follower follower;
    private final DriveTrainHardware drive;
    private static final double[] FORWARD_DELTAS = {-0.15, -0.05, 0.0, 0.05, 0.15};
    private static final double[] STRAFE_DELTAS  = {-0.15, -0.05, 0.0, 0.05, 0.15};
    private static final double[] TURN_DELTAS    = {-0.10, -0.03, 0.0, 0.03, 0.10};
    private static final double LOOKAHEAD_TIME = 0.1;
    private double maxSpeedForward = 40.0;   // adaptive
    private double maxSpeedStrafe = 30.0;   // adaptive
    private double maxTurnRate = Math.PI; // adaptive
    private static final double ACCEL_FACTOR_FORWARD = 0.3; // tune
    private static final double ACCEL_FACTOR_STRAFE  = 0.3; // tune
    private static final double TURN_COUPLING_FACTOR = 0.3; // tune
    private static final double HEADING_WEIGHT    = 5.0;   // tune
    private static final double SMOOTHNESS_WEIGHT = 0.5;   // tune
    private final Telemetry telemetry;
    private double lastBestForwardPower = 0;
    private double lastBestStrafePower = 0;
    private double lastBestTurnPower = 0;
    // SysID state — predictions saved from previous loop for comparison this loop
    private double lastPredictedForwardVel = 0;
    private double lastPredictedStrafeVel  = 0;
    private double lastPredictedTurnVel    = 0;
    private boolean haveLastPrediction = false;   // false until first prediction is made
    private static final double LEARNING_RATE = 0.01;   // how fast constants adapt; small = stable
    public LightweightMPC(Follower follower, DriveTrainHardware drive, Telemetry telemetry) {
        this.follower = follower;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    public void update() {
        // step1
        telemetry.addData("MPC", true);
        telemetry.addData("maxSpeedFwd",   maxSpeedForward);
        telemetry.addData("maxSpeedStrafe", maxSpeedStrafe);
        telemetry.addData("maxTurnRate",   maxTurnRate);
        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();
        double currentHeading = follower.getPose().getHeading();
        //step2
        double fl = drive.frontLeftDrive.getPower();
        double bl = drive.backLeftDrive.getPower();
        double fr = drive.frontRightDrive.getPower();
        double br = drive.backRightDrive.getPower();
        double baseForward = (fl + bl + fr + br) / 4;
        double baseStrafe = (fl - bl - fr + br) / 4;
        double baseTurn = (fl + bl - fr - br) / 4;
        double fieldVelX = follower.getVelocity().getXComponent();
        double fieldVelY = follower.getVelocity().getYComponent();
        double currentForwardVelocity =  fieldVelX * Math.cos(currentHeading) + fieldVelY * Math.sin(currentHeading);
        double currentStrafeVelocity  = -fieldVelX * Math.sin(currentHeading) + fieldVelY * Math.cos(currentHeading);
        //SysID: compare last loop's prediction to actual measurements now
        if (haveLastPrediction) {
            double forwardVelError = currentForwardVelocity - lastPredictedForwardVel;
            double strafeVelError  = currentStrafeVelocity  - lastPredictedStrafeVel;
            double turnVelError    = follower.getAngularVelocity() - lastPredictedTurnVel;
            if (Math.abs(lastBestForwardPower) > 0.1 && Math.abs(currentForwardVelocity) > 3.0) {
                double denom = lastBestForwardPower * ACCEL_FACTOR_FORWARD;
                maxSpeedForward += LEARNING_RATE * forwardVelError / denom;
            }
            if (Math.abs(lastBestStrafePower) > 0.1 && Math.abs(currentStrafeVelocity) > 3.0) {
                double denom = lastBestStrafePower * ACCEL_FACTOR_STRAFE;
                maxSpeedStrafe += LEARNING_RATE * strafeVelError / denom;
            }
            if (Math.abs(lastBestTurnPower) > 0.1 && Math.abs(follower.getAngularVelocity()) > 0.3) {
                maxTurnRate += LEARNING_RATE * turnVelError / lastBestTurnPower;
            }
            maxSpeedForward = Math.max(10.0, Math.min(100.0, maxSpeedForward));
            maxSpeedStrafe  = Math.max(5.0,  Math.min(80.0,  maxSpeedStrafe));
            maxTurnRate     = Math.max(0.5,  Math.min(15.0,  maxTurnRate));
        }
        double bestScore = Double.MAX_VALUE;
        double bestForwardPower = baseForward;
        double bestStrafePower = baseStrafe;
        double bestTurnPower = baseTurn;
        //step5
        double lookAheadDistance = follower.getVelocity().getMagnitude() * LOOKAHEAD_TIME;
        Path currentPath = follower.getCurrentPath();
        double currentT  = follower.getCurrentTValue();
        double targetT = Math.min(currentT + lookAheadDistance / currentPath.length(), 1.0);
        Pose targetPose = currentPath.getPose(targetT);
        double targetX = targetPose.getX();
        double targetY = targetPose.getY();
        double targetHeading = targetPose.getHeading();
        //step3
        for (double forwardDelta : FORWARD_DELTAS) {
            for (double strafeDelta : STRAFE_DELTAS) {
                for (double turnDelta : TURN_DELTAS) {
                    double forwardPower = baseForward + forwardDelta;
                    double strafePower = baseStrafe + strafeDelta;
                    double turnPower = baseTurn + turnDelta;
                    double maxWheel = Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower);
                    if (maxWheel > 1.0) {
                        forwardPower /= maxWheel;
                        strafePower /= maxWheel;
                        turnPower /= maxWheel;
                    }
                    double distanceMovedForward = currentForwardVelocity * LOOKAHEAD_TIME + (forwardPower * maxSpeedForward - currentForwardVelocity) * LOOKAHEAD_TIME * ACCEL_FACTOR_FORWARD;
                    double distanceMovedStrafe = currentStrafeVelocity * LOOKAHEAD_TIME + (strafePower * maxSpeedStrafe - currentStrafeVelocity) * LOOKAHEAD_TIME * ACCEL_FACTOR_STRAFE;
                    double turnEffect = Math.abs(turnPower);
                    double turnScale  = 1.0 - (turnEffect * TURN_COUPLING_FACTOR);
                    distanceMovedForward *= turnScale;
                    distanceMovedStrafe  *= turnScale;
                    double predictedRotation = turnPower * maxTurnRate * LOOKAHEAD_TIME;
                    double midpointHeading = currentHeading + (predictedRotation / 2.0);
                    double xMoved = (distanceMovedForward * Math.cos(midpointHeading)) - (distanceMovedStrafe * Math.sin(midpointHeading));
                    double yMoved = (distanceMovedForward * Math.sin(midpointHeading)) + (distanceMovedStrafe * Math.cos(midpointHeading));
                    double xPredicted = currentX + xMoved;
                    double yPredicted = currentY + yMoved;
                    double headingPredicted = currentHeading + predictedRotation;
                    //step5
                    double errorX = targetX - xPredicted;
                    double errorY = targetY - yPredicted;
                    double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);
                    double headingError = targetHeading - headingPredicted;
                    while (headingError >  Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;
                    headingError = Math.abs(headingError);
                    double commandChange = Math.abs(forwardPower - lastBestForwardPower) + Math.abs(strafePower  - lastBestStrafePower) + Math.abs(turnPower - lastBestTurnPower);
                    double currentScore = distanceError + headingError * HEADING_WEIGHT + commandChange * SMOOTHNESS_WEIGHT;
                    //step6
                    if (currentScore < bestScore) {
                        bestScore        = currentScore;
                        bestForwardPower = forwardPower;
                        bestStrafePower  = strafePower;
                        bestTurnPower    = turnPower;
                    }


                    // candidate body goes here — saturation, predict, score
                    // we'll fill this in step by step

                }
            }
        }
        //step7
        if (Double.isNaN(bestForwardPower) || Double.isInfinite(bestForwardPower) || Double.isNaN(bestStrafePower)  || Double.isInfinite(bestStrafePower) || Double.isNaN(bestTurnPower)    || Double.isInfinite(bestTurnPower)) {
            bestForwardPower = baseForward;
            bestStrafePower = baseStrafe;
            bestTurnPower = baseTurn;
        }
        double flPower = bestForwardPower + bestStrafePower + bestTurnPower;
        double blPower = bestForwardPower - bestStrafePower + bestTurnPower;
        double frPower = bestForwardPower - bestStrafePower - bestTurnPower;
        double brPower = bestForwardPower + bestStrafePower - bestTurnPower;
        drive.frontLeftDrive.setPower(flPower);
        drive.backLeftDrive.setPower(blPower);
        drive.frontRightDrive.setPower(frPower);
        drive.backRightDrive.setPower(brPower);
        telemetry.addData("fl", flPower);
        telemetry.addData("bl", blPower);
        telemetry.addData("fr", frPower);
        telemetry.addData("br", brPower);
        //SysID: predict what velocity we expect to see next loop
        double bestMaxWheel = Math.abs(bestForwardPower) + Math.abs(bestStrafePower) + Math.abs(bestTurnPower);
        double satF = bestForwardPower, satS = bestStrafePower, satT = bestTurnPower;
        if (bestMaxWheel > 1.0) {
            satF /= bestMaxWheel;
            satS /= bestMaxWheel;
            satT /= bestMaxWheel;
        }
        lastPredictedForwardVel = currentForwardVelocity + (satF * maxSpeedForward - currentForwardVelocity) * ACCEL_FACTOR_FORWARD;
        lastPredictedStrafeVel = currentStrafeVelocity  + (satS * maxSpeedStrafe  - currentStrafeVelocity)  * ACCEL_FACTOR_STRAFE;
        lastPredictedTurnVel = satT * maxTurnRate;
        haveLastPrediction = true;
        lastBestForwardPower = bestForwardPower;
        lastBestStrafePower = bestStrafePower;
        lastBestTurnPower = bestTurnPower;




        // run the 125-candidate loop, score them, find the best

        // mix and send to motors at the end

    }
}