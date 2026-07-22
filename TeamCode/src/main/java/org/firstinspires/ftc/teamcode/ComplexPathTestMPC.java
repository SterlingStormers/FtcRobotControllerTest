package org.firstinspires.ftc.teamcode;
import com.slipstream.Slipstream;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Complex Path Test MPC", group = "Autonomous")
@Configurable
public class ComplexPathTestMPC extends OpMode {
    public static boolean useSlipstream = true;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Slipstream slip;
    private int pathState;
    private Paths paths;
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;
    public int pos = 0;
    public static double waitTime = 1;
    private char ball1 = 'P', ball2 = 'G', ball3 = 'P';
    private char detectedBall1, detectedBall2, detectedBall3;
    private boolean slot0 = false, slot1 = false, slot2 = false;
    private DcMotor motor;
    private boolean kickerUp = false;
    private double kickerPos = 0;
    private double kickerStartTime = 0.0;
    public boolean has180Occured = false;
    private ElapsedTime runtime = new ElapsedTime();
    ColorSensingAuto colorScanner;
    public boolean ShooterSpinup = false;
    private int seenAprilTag = -1;
    private int aprilTagConfirmCount = 0;
    private static final int APRILTAG_CONFIRM_THRESHOLD = 3;
    private final int[] targetTags = {1, 2, 3};
    public double EncoderZero;

    private double autoStartTime = -1;
    private double path1EndTime = -1;
    private double path2EndTime = -1;
    private double path3EndTime = -1;
    private double path4EndTime = -1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        slip = new Slipstream(follower, hardwareMap, new SlipstreamConstants());
        slip.useSlipstream = true;
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        detectedBall3 = 'U';
        detectedBall2 = 'U';
        detectedBall1 = 'U';

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        drive.kicker.setPosition(0);
        pos = drive.intakeMotor.getCurrentPosition();
        kickerPos = drive.kicker.getPosition();
        colorScanner = new ColorSensingAuto(this, "Webcam 1");

        drive.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        drive.light.setPosition(1);

        Drawing.init();

        try { Thread.sleep(500); } catch (InterruptedException ignored) {}
        follower.updatePose();

        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        slip.update();

        pathState = autonomousPathUpdate();
        colorScanner.update();

        Drawing.drawPath(paths.Path1, new com.bylazar.field.Style("", "#00FF00", 1));
        Drawing.drawPoseHistory(follower.getPoseHistory());
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

//        panelsTelemetry.debug("Mode", useSlipstream ? "Slipstream" : "Pedro");
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("currentT", slip.ampc.currentT);
//        panelsTelemetry.debug("desired V", "(" + slip.ampc.desiredVx + ", " + slip.ampc.desiredVy + ", " + slip.ampc.desiredOmega + ")");
//        panelsTelemetry.debug("maxSpeedForward", slip.ampc.maxSpeedForward);
//        panelsTelemetry.debug("maxSpeedStrafe", slip.ampc.maxSpeedStrafe);
//        panelsTelemetry.debug("maxTurnRateRad", slip.ampc.maxTurnRateRad);
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(56.000, 8.000),
                            new Pose(57.724, 37.697),
                            new Pose(39.203, 35.300)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(39.203, 35.300),
                            new Pose(12.694, 35.464)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(12.694, 35.464),
                            new Pose(32.338, 35.974),
                            new Pose(51.160, 29.975),
                            new Pose(47.808, 59.380)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(47.808, 59.380),
                            new Pose(12.576, 59.243)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
        }
    }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();

        if (newState == 10 || newState == 11 || newState == 12) {
            kickerUp = false;
        }
    }

    public int autonomousPathUpdate() {
        if (pathState == -1) {
            telemetry.addData("=== TIMING ===", "");
            telemetry.addData("Path 1 time", "%.3f", path1EndTime - autoStartTime);
            telemetry.addData("Path 2 time", "%.3f", path2EndTime - path1EndTime);
            telemetry.addData("Path 3 time", "%.3f", path3EndTime - path2EndTime);
            telemetry.addData("Path 4 time", "%.3f", path4EndTime - path3EndTime);
            telemetry.addData("Total time",  "%.3f", path4EndTime - autoStartTime);
            telemetry.update();
            drive.shooterMotor.setPower(0);
            drive.spindexer.setPower(0);
            drive.intakeMotor.setPower(0);
            return -1;
        }

        switch (pathState) {
            case 0:
                drive.intakeMotor.setPower(0);
                drive.shooterMotor.setPower(0);
                drive.spindexer.setPower(0);
                setPathState(1);
                break;

            case 1:
                if (autoStartTime < 0) autoStartTime = runtime.seconds();
                follower.followPath(paths.Path1);
                setPathState(2);
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path1EndTime = runtime.seconds();
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path2EndTime = runtime.seconds();
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path3EndTime = runtime.seconds();
                    follower.followPath(paths.Path4);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathState != -1 && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path4EndTime = runtime.seconds();
                    telemetry.addData("=== TIMING ===", "");
                    telemetry.addData("Path 1 time", "%.3f", path1EndTime - autoStartTime);
                    telemetry.addData("Path 2 time", "%.3f", path2EndTime - path1EndTime);
                    telemetry.addData("Path 3 time", "%.3f", path3EndTime - path2EndTime);
                    telemetry.addData("Path 4 time", "%.3f", path4EndTime - path3EndTime);
                    telemetry.addData("Total time",  "%.3f", path4EndTime - autoStartTime);
                    telemetry.update();
                    drive.intakeMotor.setPower(0);
                    drive.shooterMotor.setPower(0);
                    drive.spindexer.setPower(0);
                    setPathState(-1);
                }
                break;
        }
        return pathState;
    }
}