package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SlipstreamTuning.follower;
import static org.firstinspires.ftc.teamcode.SlipstreamTuning.motors;
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

@Configurable
@TeleOp(name = "Slipstream Tuning", group = "Slipstream")
public class SlipstreamTuning extends SelectableOpMode {

    public static Follower follower;
    public static DcMotor[] motors;
    @IgnoreConfigurable
    public static TelemetryManager panel;

    public SlipstreamTuning() {super("Select a Slipstream Tuning OpMode", s -> {s.folder("Automatic", a -> {a.add("Max Speed Forward Test", MaxSpeedForwardTest::new);a.add("Max Speed Strafe Test", MaxSpeedStrafeTest::new);a.add("Max Turn Rate Test", MaxTurnRateTest::new);});});}

    @Override
    public void onSelect() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        motors = new DcMotor[]{hardwareMap.get(DcMotor.class, SlipstreamConstants.leftFrontMotorName), hardwareMap.get(DcMotor.class, SlipstreamConstants.rightFrontMotorName), hardwareMap.get(DcMotor.class, SlipstreamConstants.leftBackMotorName), hardwareMap.get(DcMotor.class, SlipstreamConstants.rightBackMotorName)};

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
        if (stopping) {
            stopMotors();
            return;
        }
        if (gamepad1.bWasPressed()) {
            stopMotors();
            stopping = true;
            return;
        }

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
        if (stopping) {
            stopMotors();
            return;
        }
        if (gamepad1.bWasPressed()) {
            stopMotors();
            stopping = true;
            return;
        }

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

class MaxTurnRateTest extends OpMode {
    public static double TARGET_ROTATIONS = 3.0;   // number of full 2π turns
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
        if (stopping) {
            stopMotors();
            return;
        }
        if (gamepad1.bWasPressed()) {
            stopMotors();
            stopping = true;
            return;
        }

        follower.updatePose();
        double turnedRadians = Math.abs(follower.getTotalHeading() - startHeading);
        double targetRadians = TARGET_ROTATIONS * 2 * Math.PI;

        if (measuring && turnedRadians >= targetRadians) {
            stopMotors();
            measuring = false;
        }

        if (measuring) {
            // Counterclockwise turn: left side backward, right side forward
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