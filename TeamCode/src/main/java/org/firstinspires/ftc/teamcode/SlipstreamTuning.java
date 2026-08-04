package org.firstinspires.ftc.teamcode;
import com.slipstream.AMPC;
import com.slipstream.VelocityController;
import com.slipstream.MecanumKinematics;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.follower;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.panel;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.setPowers;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.stopMotors;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.constants;
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

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/19/2026
 */

@Configurable
@TeleOp(name = "Slipstream Tuning", group = "Slipstream")
public class SlipstreamTuning extends SelectableOpMode {
    public static Follower follower;
    public static DcMotor[] motors;
    public static SlipstreamConstants constants;
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
        constants = new SlipstreamConstants();
        follower.setStartingPose(new Pose(0, 0, 0));

        motors = new DcMotor[]{
                hardwareMap.get(DcMotor.class, constants.leftFrontMotorName),
                hardwareMap.get(DcMotor.class, constants.rightFrontMotorName),
                hardwareMap.get(DcMotor.class, constants.leftBackMotorName),
                hardwareMap.get(DcMotor.class, constants.rightBackMotorName)
        };

        motors[0].setDirection(constants.leftFrontDirection);
        motors[1].setDirection(constants.rightFrontDirection);
        motors[2].setDirection(constants.leftBackDirection);
        motors[3].setDirection(constants.rightBackDirection);

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
    private static final double[] POWER_FACTORS = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
    private static final double DRIVE_TIME_SECONDS = 1.0;
    private static final double BRAKE_TIMEOUT_SECONDS = 3.0;
    private static final double STOPPED_THRESHOLD = 1.0;
    private static final double PAUSE_BETWEEN_SECONDS = 1.0;
    private enum Phase { ACCEL, BRAKE, PAUSE, DONE }
    private Phase phase = Phase.ACCEL;
    private int currentTrial = 0;
    private double direction = 1.0;
    private long phaseStartNs;
    private int belowThresholdCount = 0;
    private double measuredCruiseVel;
    private double brakeStartX, brakeStartY;
    private SlipstreamConstants config;
    private AMPC ampc;
    private VelocityController controller;
    private MecanumKinematics kinematics;

    private static class TrialResult {
        double peakVelocity;
        double brakingDistance;
        TrialResult(double v, double d) { peakVelocity = v; brakingDistance = d; }
    }

    private final java.util.List<TrialResult> trialResults = new java.util.ArrayList<>();
    private boolean stopping = false;

    @Override
    public void init() {
        config = new SlipstreamConstants();
        ampc = new AMPC(follower, config);
        controller = new VelocityController(follower, ampc, config);
        kinematics = new MecanumKinematics(hardwareMap, ampc, controller, config);
    }

    @Override
    public void init_loop() {
        panel.debug("Max Decel Test - Adaptive Predictive Braking Calibration");
        panel.debug("Runs " + POWER_FACTORS.length + " trials at varied peak speeds.");
        panel.debug("Drives via VelocityController, then brakes via VelocityController.");
        panel.debug("Trials alternate forward/reverse.");
        panel.debug("Results -> kLinearBraking, kQuadraticFriction in SlipstreamConstants.");
        panel.debug("IMPORTANT: Fully charged battery. 5+ feet clearance both directions.");
        panel.debug("If you tune PIDF later, rerun this test.");
        panel.debug("B on gamepad 1: stop");
        panel.update(telemetry);
        follower.updatePose();
    }

    @Override
    public void start() {
        follower.updatePose();
        controller.reset();
        currentTrial = 0;
        phase = Phase.ACCEL;
        beginAccel();
    }

    private void beginAccel() {
        direction = (currentTrial % 2 == 0) ? 1.0 : -1.0;
        phaseStartNs = System.nanoTime();
        belowThresholdCount = 0;
        controller.reset();
    }

    private void commandStop() {
        ampc.desiredVx = 0;
        ampc.desiredVy = 0;
        ampc.desiredOmega = 0;
        controller.velocity();
        kinematics.drive();
    }

    @Override
    public void loop() {
        if (stopping) { commandStop(); return; }
        if (gamepad1.bWasPressed()) { commandStop(); stopping = true; return; }

        follower.updatePose();
        double elapsed = (System.nanoTime() - phaseStartNs) / 1e9;

        double fieldVx = follower.getVelocity().getXComponent();
        double fieldVy = follower.getVelocity().getYComponent();
        double absV = Math.hypot(fieldVx, fieldVy);

        switch (phase) {
            case ACCEL: {
                if (currentTrial >= POWER_FACTORS.length) {
                    phase = Phase.DONE;
                    break;
                }
                double powerFactor = POWER_FACTORS[currentTrial];
                double targetVx = direction * powerFactor * config.maxSpeedForward;

                ampc.desiredVx = targetVx;
                ampc.desiredVy = 0;
                ampc.desiredOmega = 0;
                controller.velocity();
                kinematics.drive();

                if (elapsed >= DRIVE_TIME_SECONDS) {
                    measuredCruiseVel = absV;
                    brakeStartX = follower.getPose().getX();
                    brakeStartY = follower.getPose().getY();
                    phaseStartNs = System.nanoTime();
                    phase = Phase.BRAKE;
                }
                break;
            }

            case BRAKE: {
                ampc.desiredVx = 0;
                ampc.desiredVy = 0;
                ampc.desiredOmega = 0;
                controller.velocity();
                kinematics.drive();

                if (absV <= STOPPED_THRESHOLD) belowThresholdCount++;
                else belowThresholdCount = 0;

                boolean stopped = belowThresholdCount >= 5 && elapsed > 0.2;
                boolean brakeTimeout = elapsed > BRAKE_TIMEOUT_SECONDS;

                if (stopped || brakeTimeout) {
                    double dx = follower.getPose().getX() - brakeStartX;
                    double dy = follower.getPose().getY() - brakeStartY;
                    double brakingDistance = Math.hypot(dx, dy);

                    if (brakingDistance > 0.1 && measuredCruiseVel > 3.0) {
                        trialResults.add(new TrialResult(measuredCruiseVel, brakingDistance));
                    }

                    currentTrial++;
                    phaseStartNs = System.nanoTime();
                    phase = Phase.PAUSE;
                }
                break;
            }

            case PAUSE: {
                commandStop();
                if (elapsed >= PAUSE_BETWEEN_SECONDS) {
                    if (currentTrial >= POWER_FACTORS.length) {
                        phase = Phase.DONE;
                    } else {
                        beginAccel();
                        phase = Phase.ACCEL;
                    }
                }
                break;
            }

            case DONE: {
                commandStop();

                if (trialResults.size() < 3) {
                    panel.debug("ERROR: Only " + trialResults.size() + " valid trials. Rerun test.");
                    panel.update(telemetry);
                    break;
                }

                double S1 = 0, S2 = 0, S3 = 0, T1 = 0, T2 = 0;
                for (TrialResult r : trialResults) {
                    double v = r.peakVelocity;
                    double v2 = v * v;
                    S1 += v2;
                    S2 += v * v2;
                    S3 += v2 * v2;
                    T1 += v * r.brakingDistance;
                    T2 += v2 * r.brakingDistance;
                }
                double det = S1 * S3 - S2 * S2;
                if (Math.abs(det) < 1e-9) {
                    panel.debug("ERROR: Fit failed (singular matrix). Rerun test.");
                    panel.update(telemetry);
                    break;
                }
                double kLinear = (T1 * S3 - T2 * S2) / det;
                double kQuadratic = (S1 * T2 - S2 * T1) / det;

                double meanDist = 0;
                for (TrialResult r : trialResults) meanDist += r.brakingDistance;
                meanDist /= trialResults.size();

                double sumResidualSq = 0, sumTotalSq = 0;
                for (TrialResult r : trialResults) {
                    double predicted = kLinear * r.peakVelocity + kQuadratic * r.peakVelocity * r.peakVelocity;
                    sumResidualSq += (r.brakingDistance - predicted) * (r.brakingDistance - predicted);
                    sumTotalSq += (r.brakingDistance - meanDist) * (r.brakingDistance - meanDist);
                }
                double rSquared = sumTotalSq > 0 ? 1.0 - sumResidualSq / sumTotalSq : 0;

                panel.debug("Adaptive Predictive Braking Results");
                panel.debug("Trials collected: " + trialResults.size());
                for (int i = 0; i < trialResults.size(); i++) {
                    TrialResult r = trialResults.get(i);
                    panel.debug(String.format("Trial %d: v=%.2f in/s, d=%.2f in", i, r.peakVelocity, r.brakingDistance));
                }
                panel.debug("kLinearBraking: " + String.format("%.4f", kLinear));
                panel.debug("kQuadraticFriction: " + String.format("%.6f", kQuadratic));
                panel.debug("Fit R^2: " + String.format("%.3f", rSquared));
                if (rSquared < 0.9) panel.debug("WARNING: R^2 below 0.9, fit quality is poor.");
                if (kLinear < 0 || kQuadratic < 0) panel.debug("WARNING: Negative coefficient detected.");
                panel.debug("Copy to SlipstreamConstants:");
                panel.debug("  kLinearBraking = " + String.format("%.4f", kLinear));
                panel.debug("  kQuadraticFriction = " + String.format("%.6f", kQuadratic));
                panel.update(telemetry);
                break;
            }
        }

        panel.debug("Phase: " + phase + " Trial: " + (currentTrial + 1) + "/" + POWER_FACTORS.length);
        panel.debug("Direction: " + (direction > 0 ? "FORWARD" : "REVERSE"));
        panel.debug("Speed: " + String.format("%.2f", absV));
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

        double effort = constants.vxKf * desired + constants.vxKp * error + constants.vxKi * integral + constants.vxKd * derivative;

        double norm = effort / constants.maxSpeedForward;
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

        double effort = constants.vyKf * desired + constants.vyKp * error + constants.vyKi * integral + constants.vyKd * derivative;

        double norm = effort / constants.maxSpeedStrafe;
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

        double effort = constants.omegaKf * desired + constants.omegaKp * error + constants.omegaKi * integral + constants.omegaKd * derivative;

        double norm = effort / constants.maxTurnRate;
        norm = Math.max(-1.0, Math.min(1.0, norm));
        setPowers(-norm, norm, -norm, norm);

        panel.debug("Desired Omega: " + desired);
        panel.debug("Actual Omega: " + actualOmega);
        panel.addData("Zero Line", 0);
        panel.addData("Error", error);
        panel.update(telemetry);
    }
}