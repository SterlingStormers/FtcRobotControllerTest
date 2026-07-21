package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.follower;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.panel;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.setPowers;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.stopMotors;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

//use these constants for later
//package com.slipstream.tuning;
//
//import static com.slipstream.tuning.SlipstreamTuning.follower;
//import static com.slipstream.tuning.SlipstreamTuning.panel;
//import static com.slipstream.tuning.SlipstreamTuning.setPowers;
//import static com.slipstream.tuning.SlipstreamTuning.stopMotors;
//
//import com.slipstream.SlipstreamConstants;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.telemetry.SelectableOpMode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import java.util.List;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

@Configurable
@TeleOp(name = "Slipstream Tuning", group = "Slipstream")
public class SlipstreamTuning extends SelectableOpMode {

    public static Follower follower;
    public static DcMotor[] motors;
    @IgnoreConfigurable
    public static TelemetryManager panel;

    public SlipstreamTuning() {
        super("Select a Slipstream Tuning OpMode", s -> {
            s.folder("Automatic", a -> {
                a.add("Max Speed Forward Test", MaxSpeedForwardTest::new);
                a.add("Max Speed Strafe Test", MaxSpeedStrafeTest::new);
                a.add("Max Turn Rate Test", MaxTurnRateTest::new);
                a.add("Max Decel Test", MaxDecelTest::new);
            });
            s.folder("PIDF", p -> {
                p.add("Vx PIDF Tuner", VxPIDFTuner::new);
                p.add("Vy PIDF Tuner", VyPIDFTuner::new);
                p.add("Omega PIDF Tuner", OmegaPIDFTuner::new);
            });
        });
    }

    @Override
    public void onSelect() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        motors = new DcMotor[]{
                hardwareMap.get(DcMotor.class, SlipstreamConstants.leftFrontMotorName),
                hardwareMap.get(DcMotor.class, SlipstreamConstants.rightFrontMotorName),
                hardwareMap.get(DcMotor.class, SlipstreamConstants.leftBackMotorName),
                hardwareMap.get(DcMotor.class, SlipstreamConstants.rightBackMotorName)
        };

        motors[0].setDirection(SlipstreamConstants.leftFrontDirection);
        motors[1].setDirection(SlipstreamConstants.rightFrontDirection);
        motors[2].setDirection(SlipstreamConstants.leftBackDirection);
        motors[3].setDirection(SlipstreamConstants.rightBackDirection);

        for (DcMotor m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        panel = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void setPowers(double fl, double fr, double bl, double br) {
        motors[0].setPower(fl);
        motors[1].setPower(fr);
        motors[2].setPower(bl);
        motors[3].setPower(br);
    }

    public static void stopMotors() {
        setPowers(0, 0, 0, 0);
    }
}

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

class MaxSpeedForwardTest extends OpMode {
    public static double TARGET_DISTANCE = 48;
    public static int SAMPLE_WINDOW = 10;
    private final double[] recentSpeeds = new double[SAMPLE_WINDOW];
    private int sampleIndex = 0;
    private double startX;
    private boolean measuring = true;
    private boolean stopping = false;

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        panel.debug("Max Speed Forward Test");
        panel.debug("Runs the robot forward at full power for " + TARGET_DISTANCE + " inches.");
        panel.debug("Averages the last " + SAMPLE_WINDOW + " velocity samples during cruise.");
        panel.debug("Result -> SlipstreamConstants.maxSpeedForward");
        panel.debug("IMPORTANT: Use a fully charged battery for accurate results.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        follower.updatePose();
        startX = follower.getPose().getX();
    }

    @Override
    public void loop() {
        if (stopping) { stopMotors(); return; }
        if (gamepad1.bWasPressed()) { stopMotors(); stopping = true; return; }

        follower.updatePose();
        double distanceCovered = Math.abs(follower.getPose().getX() - startX);

        if (measuring && distanceCovered >= TARGET_DISTANCE) {
            stopMotors();
            measuring = false;
        }

        if (measuring) {
            setPowers(1.0, 1.0, 1.0, 1.0);
            recentSpeeds[sampleIndex] = Math.abs(follower.getVelocity().getXComponent());
            sampleIndex = (sampleIndex + 1) % SAMPLE_WINDOW;
        } else {
            double sum = 0;
            for (double s : recentSpeeds) sum += s;
            double result = sum / SAMPLE_WINDOW;
            panel.debug("Max Forward Velocity: " + result + " in/s");
            panel.debug("Distance covered: " + distanceCovered + " inches");
            panel.debug("Copy value to SlipstreamConstants.maxSpeedForward");
            panel.update(telemetry);
        }
    }
}

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

class MaxSpeedStrafeTest extends OpMode {
    public static double TARGET_DISTANCE = 48;
    public static int SAMPLE_WINDOW = 10;
    private final double[] recentSpeeds = new double[SAMPLE_WINDOW];
    private int sampleIndex = 0;
    private double startY;
    private boolean measuring = true;
    private boolean stopping = false;

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        panel.debug("Max Speed Strafe Test");
        panel.debug("Runs the robot right (strafe) at full power for " + TARGET_DISTANCE + " inches.");
        panel.debug("Averages the last " + SAMPLE_WINDOW + " velocity samples during cruise.");
        panel.debug("Result -> SlipstreamConstants.maxSpeedStrafe");
        panel.debug("IMPORTANT: Use a fully charged battery for accurate results.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        follower.updatePose();
        startY = follower.getPose().getY();
    }

    @Override
    public void loop() {
        if (stopping) { stopMotors(); return; }
        if (gamepad1.bWasPressed()) { stopMotors(); stopping = true; return; }

        follower.updatePose();
        double distanceCovered = Math.abs(follower.getPose().getY() - startY);

        if (measuring && distanceCovered >= TARGET_DISTANCE) {
            stopMotors();
            measuring = false;
        }

        if (measuring) {
            setPowers(1.0, -1.0, -1.0, 1.0);
            recentSpeeds[sampleIndex] = Math.abs(follower.getVelocity().getYComponent());
            sampleIndex = (sampleIndex + 1) % SAMPLE_WINDOW;
        } else {
            double sum = 0;
            for (double s : recentSpeeds) sum += s;
            double result = sum / SAMPLE_WINDOW;
            panel.debug("Max Strafe Velocity: " + result + " in/s");
            panel.debug("Distance covered: " + distanceCovered + " inches");
            panel.debug("Copy value to SlipstreamConstants.maxSpeedStrafe");
            panel.update(telemetry);
        }
    }
}

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

class MaxTurnRateTest extends OpMode {
    public static double TARGET_ROTATIONS = 3.0;
    public static int SAMPLE_WINDOW = 10;
    private final double[] recentOmegas = new double[SAMPLE_WINDOW];
    private int sampleIndex = 0;
    private double startHeading;
    private boolean measuring = true;
    private boolean stopping = false;

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        panel.debug("Max Turn Rate Test");
        panel.debug("Spins the robot counterclockwise at full power for " + TARGET_ROTATIONS + " full rotations.");
        panel.debug("Averages the last " + SAMPLE_WINDOW + " angular velocity samples during cruise.");
        panel.debug("Result -> SlipstreamConstants.maxTurnRate");
        panel.debug("IMPORTANT: Use a fully charged battery for accurate results.");
        panel.debug("Ensure at least 3 feet of clearance around the robot.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        follower.updatePose();
        startHeading = follower.getTotalHeading();
    }

    @Override
    public void loop() {
        if (stopping) { stopMotors(); return; }
        if (gamepad1.bWasPressed()) { stopMotors(); stopping = true; return; }

        follower.updatePose();
        double turnedRadians = Math.abs(follower.getTotalHeading() - startHeading);
        double targetRadians = TARGET_ROTATIONS * 2 * Math.PI;

        if (measuring && turnedRadians >= targetRadians) {
            stopMotors();
            measuring = false;
        }

        if (measuring) {
            setPowers(-1.0, 1.0, -1.0, 1.0);
            recentOmegas[sampleIndex] = Math.abs(follower.getAngularVelocity());
            sampleIndex = (sampleIndex + 1) % SAMPLE_WINDOW;
        } else {
            double sum = 0;
            for (double o : recentOmegas) sum += o;
            double result = sum / SAMPLE_WINDOW;
            panel.debug("Max Turn Rate: " + result + " rad/s");
            panel.debug("Rotations completed: " + (turnedRadians / (2 * Math.PI)));
            panel.debug("Copy value to SlipstreamConstants.maxTurnRate");
            panel.update(telemetry);
        }
    }
}

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

class MaxDecelTest extends OpMode {
    public static double ACCEL_TIME_SECONDS = 1.5;
    public static double PAUSE_BETWEEN_SECONDS = 1.5;
    public static int NUM_TRIALS = 3;
    public static double STOPPED_THRESHOLD = 1.0;
    private enum Phase { ACCEL, BRAKE, PAUSE, DONE }
    private Phase phase = Phase.ACCEL;
    private int currentTrial = 0;
    private double direction = 1.0;
    private long phaseStartNs;
    private double cruiseVelSum = 0;
    private int cruiseVelCount = 0;
    private double lastX;
    private long lastSampleNs;
    private double measuredCruiseVel;
    private double brakeStartX;
    private final double[] decelMeasurements = new double[NUM_TRIALS];
    private boolean stopping = false;

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        panel.debug("Max Decel Test");
        panel.debug("Runs " + NUM_TRIALS + " trials alternating forward/reverse.");
        panel.debug("Accelerates for " + ACCEL_TIME_SECONDS + " seconds, then actively brakes at full reverse power.");
        panel.debug("Result -> SlipstreamConstants.maxDecel");
        panel.debug("IMPORTANT: Use a fully charged battery for accurate results.");
        panel.debug("Ensure at least 5 feet of clearance in both directions.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        follower.updatePose();
        currentTrial = 0;
        phase = Phase.ACCEL;
        beginAccel();
    }

    private void beginAccel() {
        direction = (currentTrial % 2 == 0) ? 1.0 : -1.0;
        phaseStartNs = System.nanoTime();
        cruiseVelSum = 0;
        cruiseVelCount = 0;
        lastX = follower.getPose().getX();
        lastSampleNs = phaseStartNs;
    }

    @Override
    public void loop() {
        if (stopping) { stopMotors(); return; }
        if (gamepad1.bWasPressed()) { stopMotors(); stopping = true; return; }

        follower.updatePose();
        double elapsed = (System.nanoTime() - phaseStartNs) / 1e9;
        double currentX = follower.getPose().getX();

        switch (phase) {
            case ACCEL: {
                double power = direction;
                setPowers(power, power, power, power);
                long now = System.nanoTime();
                double sampleDt = (now - lastSampleNs) / 1e9;
                if (sampleDt > 0.02) {
                    double v = (currentX - lastX) / sampleDt;
                    if (elapsed > ACCEL_TIME_SECONDS - 0.3) {
                        cruiseVelSum += Math.abs(v);
                        cruiseVelCount++;
                    }
                    lastX = currentX;
                    lastSampleNs = now;
                }
                if (elapsed >= ACCEL_TIME_SECONDS) {
                    measuredCruiseVel = cruiseVelCount > 0 ? cruiseVelSum / cruiseVelCount : 0;
                    brakeStartX = currentX;
                    double brakePower = -direction;
                    setPowers(brakePower, brakePower, brakePower, brakePower);
                    phaseStartNs = System.nanoTime();
                    phase = Phase.BRAKE;
                }
                break;
            }
            case BRAKE: {
                long now = System.nanoTime();
                double sampleDt = (now - lastSampleNs) / 1e9;
                double v = sampleDt > 0.001 ? (currentX - lastX) / sampleDt : 0;
                lastX = currentX;
                lastSampleNs = now;
                boolean stopped = Math.abs(v) <= STOPPED_THRESHOLD && elapsed > 0.2;
                boolean brakeTimeout = elapsed > 3.0;
                if (stopped || brakeTimeout) {
                    stopMotors();
                    double brakeDist = Math.abs(currentX - brakeStartX);
                    if (brakeDist > 0.1) {
                        decelMeasurements[currentTrial] = (measuredCruiseVel * measuredCruiseVel) / (2 * brakeDist);
                    }
                    currentTrial++;
                    phaseStartNs = System.nanoTime();
                    phase = Phase.PAUSE;
                }
                break;
            }
            case PAUSE: {
                stopMotors();
                if (elapsed >= PAUSE_BETWEEN_SECONDS) {
                    if (currentTrial >= NUM_TRIALS) {
                        phase = Phase.DONE;
                    } else {
                        beginAccel();
                        phase = Phase.ACCEL;
                    }
                }
                break;
            }
            case DONE: {
                stopMotors();
                double min = decelMeasurements[0], max = decelMeasurements[0], sum = 0;
                for (double d : decelMeasurements) {
                    min = Math.min(min, d);
                    max = Math.max(max, d);
                    sum += d;
                }
                double mean = sum / decelMeasurements.length;
                double recommended = min * 0.95;
                panel.debug("Max Decel Results");
                for (int i = 0; i < decelMeasurements.length; i++) {
                    panel.debug("Trial " + (i + 1) + ": " + decelMeasurements[i] + " in/s^2");
                }
                panel.debug("Min: " + min);
                panel.debug("Max: " + max);
                panel.debug("Mean: " + mean);
                panel.debug("Recommended maxDecel: " + recommended);
                panel.debug("Copy recommended value to SlipstreamConstants.maxDecel");
                panel.update(telemetry);
                break;
            }
        }

        panel.debug("Phase: " + phase + " Trial: " + (currentTrial + 1) + "/" + NUM_TRIALS);
        panel.debug("Direction: " + (direction > 0 ? "FORWARD" : "REVERSE"));
        panel.update(telemetry);
    }
}

/**
 * Alternates between +TARGET and -TARGET forward velocity every SWITCH_INTERVAL seconds.
 * Runs inline Vx PIDF reading LIVE from SlipstreamConstants.
 * User adjusts vxKp, vxKi, vxKd, vxKf in Panels while watching Error.
 */

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

class VxPIDFTuner extends OpMode {
    public static double TARGET_VELOCITY = 25;
    public static double SWITCH_INTERVAL_SECONDS = 1.5;
    private double integral = 0;
    private double lastError = 0;
    private long lastTimeNs = 0;
    private long phaseStartMs = 0;
    private double sign = 1;
    private boolean stopping = false;

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        panel.debug("Vx PIDF Tuner");
        panel.debug("Alternates between +" + TARGET_VELOCITY + " and -" + TARGET_VELOCITY + " in/s forward every " + SWITCH_INTERVAL_SECONDS + " seconds.");
        panel.debug("Adjust vxKp, vxKi, vxKd, vxKf in Panels while watching Error.");
        panel.debug("Goal: small error, no oscillation, quick settling.");
        panel.debug("Ensure ample forward and backward clearance.");
        panel.debug("IMPORTANT: Use a fully charged battery for accurate results.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        integral = 0;
        lastError = 0;
        lastTimeNs = 0;
        phaseStartMs = System.currentTimeMillis();
        sign = 1;
    }

    @Override
    public void loop() {
        if (stopping) { stopMotors(); return; }
        if (gamepad1.bWasPressed()) { stopMotors(); stopping = true; return; }

        follower.updatePose();

        // Switch direction
        if (System.currentTimeMillis() - phaseStartMs > SWITCH_INTERVAL_SECONDS * 1000) {
            sign = -sign;
            phaseStartMs = System.currentTimeMillis();
            integral = 0;
            lastError = 0;
        }
        double desired = sign * TARGET_VELOCITY;

        double h = follower.getPose().getHeading();
        double fieldVx = follower.getVelocity().getXComponent();
        double fieldVy = follower.getVelocity().getYComponent();
        double actualVx = fieldVx * Math.cos(h) + fieldVy * Math.sin(h);

        // PIDF (live from SlipstreamConstants)
        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        double error = desired - actualVx;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double effort = SlipstreamConstants.vxKf * desired + SlipstreamConstants.vxKp * error + SlipstreamConstants.vxKi * integral + SlipstreamConstants.vxKd * derivative;

        double norm = effort / SlipstreamConstants.maxSpeedForward;
        norm = Math.max(-1.0, Math.min(1.0, norm));
        setPowers(norm, norm, norm, norm);

        panel.debug("Desired Vx: " + desired);
        panel.debug("Actual Vx: " + actualVx);
        panel.addData("Zero Line", 0);
        panel.addData("Error", error);
        panel.update(telemetry);
    }
}

/**
 * Alternates between +TARGET and -TARGET strafe velocity every SWITCH_INTERVAL seconds.
 * Runs inline Vy PIDF reading LIVE from SlipstreamConstants.
 * User adjusts vyKp, vyKi, vyKd, vyKf in Panels while watching Error.
 */

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

class VyPIDFTuner extends OpMode {
    public static double TARGET_VELOCITY = 20;
    public static double SWITCH_INTERVAL_SECONDS = 1.5;
    private double integral = 0;
    private double lastError = 0;
    private long lastTimeNs = 0;
    private long phaseStartMs = 0;
    private double sign = 1;
    private boolean stopping = false;

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        panel.debug("Vy PIDF Tuner");
        panel.debug("Alternates between +" + TARGET_VELOCITY + " and -" + TARGET_VELOCITY + " in/s strafe every " + SWITCH_INTERVAL_SECONDS + " seconds.");
        panel.debug("Adjust vyKp, vyKi, vyKd, vyKf in Panels while watching Error.");
        panel.debug("Goal: small error, no oscillation, quick settling.");
        panel.debug("Ensure ample left and right clearance.");
        panel.debug("IMPORTANT: Use a fully charged battery for accurate results.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        integral = 0;
        lastError = 0;
        lastTimeNs = 0;
        phaseStartMs = System.currentTimeMillis();
        sign = 1;
    }

    @Override
    public void loop() {
        if (stopping) { stopMotors(); return; }
        if (gamepad1.bWasPressed()) { stopMotors(); stopping = true; return; }

        follower.updatePose();

        if (System.currentTimeMillis() - phaseStartMs > SWITCH_INTERVAL_SECONDS * 1000) {
            sign = -sign;
            phaseStartMs = System.currentTimeMillis();
            integral = 0;
            lastError = 0;
        }
        double desired = sign * TARGET_VELOCITY;

        double h = follower.getPose().getHeading();
        double fieldVx = follower.getVelocity().getXComponent();
        double fieldVy = follower.getVelocity().getYComponent();
        double actualVy = -fieldVx * Math.sin(h) + fieldVy * Math.cos(h);

        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        double error = desired - actualVy;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double effort = SlipstreamConstants.vyKf * desired + SlipstreamConstants.vyKp * error + SlipstreamConstants.vyKi * integral + SlipstreamConstants.vyKd * derivative;

        double norm = effort / SlipstreamConstants.maxSpeedStrafe;
        norm = Math.max(-1.0, Math.min(1.0, norm));
        setPowers(norm, -norm, -norm, norm);

        panel.debug("Desired Vy: " + desired);
        panel.debug("Actual Vy: " + actualVy);
        panel.addData("Zero Line", 0);
        panel.addData("Error", error);
        panel.update(telemetry);
    }
}

/**
 * Alternates between +TARGET and -TARGET angular velocity every SWITCH_INTERVAL seconds.
 * Runs inline Omega PIDF reading LIVE from SlipstreamConstants.
 * User adjusts omegaKp, omegaKi, omegaKd, omegaKf in Panels while watching Error graph.
 */

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

class OmegaPIDFTuner extends OpMode {
    public static double TARGET_OMEGA = 2.0;
    public static double SWITCH_INTERVAL_SECONDS = 1.5;

    private double integral = 0;
    private double lastError = 0;
    private long lastTimeNs = 0;
    private long phaseStartMs = 0;
    private double sign = 1;
    private boolean stopping = false;

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        panel.debug("Omega PIDF Tuner");
        panel.debug("Alternates between +" + TARGET_OMEGA + " and -" + TARGET_OMEGA + " rad/s turn every " + SWITCH_INTERVAL_SECONDS + " seconds.");
        panel.debug("Adjust omegaKp, omegaKi, omegaKd, omegaKf in Panels while watching Error.");
        panel.debug("Goal: small error, no oscillation, quick settling.");
        panel.debug("Ensure at least 3 feet of clearance around the robot.");
        panel.debug("IMPORTANT: Use a fully charged battery for accurate results.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        integral = 0;
        lastError = 0;
        lastTimeNs = 0;
        phaseStartMs = System.currentTimeMillis();
        sign = 1;
    }

    @Override
    public void loop() {
        if (stopping) { stopMotors(); return; }
        if (gamepad1.bWasPressed()) { stopMotors(); stopping = true; return; }

        follower.updatePose();

        if (System.currentTimeMillis() - phaseStartMs > SWITCH_INTERVAL_SECONDS * 1000) {
            sign = -sign;
            phaseStartMs = System.currentTimeMillis();
            integral = 0;
            lastError = 0;
        }
        double desired = sign * TARGET_OMEGA;

        double actualOmega = follower.getAngularVelocity();

        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0.02 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        double error = desired - actualOmega;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double effort = SlipstreamConstants.omegaKf * desired + SlipstreamConstants.omegaKp * error + SlipstreamConstants.omegaKi * integral + SlipstreamConstants.omegaKd * derivative;

        double norm = effort / SlipstreamConstants.maxTurnRate;
        norm = Math.max(-1.0, Math.min(1.0, norm));
        setPowers(-norm, norm, -norm, norm);

        panel.debug("Desired Omega: " + desired);
        panel.debug("Actual Omega: " + actualOmega);
        panel.addData("Zero Line", 0);
        panel.addData("Error", error);
        panel.update(telemetry);
    }
}