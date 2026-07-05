package org.firstinspires.ftc.teamcode;
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
    private TelemetryManager panelsTelemetry;
    public Follower follower;
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

    // Per-path timing
    private double autoStartTime = -1;
    private double path1EndTime = -1;
    private double path2EndTime = -1;
    private double path3EndTime = -1;
    private double path4EndTime = -1;
    private double path5EndTime = -1;
    private double path6EndTime = -1;
    private AMPC mpc;
    private VelocityControllerV2 controller;
    private MecanumKinematics kinematics;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23.907, 119.235, Math.toRadians(90)));

        paths = new Paths(follower);

        detectedBall3 = 'U';
        detectedBall2 = 'U';
        detectedBall1 = 'U';

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        mpc = new AMPC(follower);
        controller = new VelocityControllerV2(follower, mpc);
        kinematics = new MecanumKinematics(drive, mpc, controller);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        drive.kicker.setPosition(0);
        pos = drive.intakeMotor.getCurrentPosition();
        kickerPos = drive.kicker.getPosition();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        colorScanner = new ColorSensingAuto(this, "Webcam 1");

        drive.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        drive.light.setPosition(1);

        try { Thread.sleep(500); } catch (InterruptedException ignored) {}
        follower.updatePose();

        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.updatePose();
        mpc.update();
        controller.velocity();
        kinematics.drive();

        pathState = autonomousPathUpdate();
        colorScanner.update();

//        if (ShooterSpinup && !follower.isBusy() && follower.getCurrentTValue() >= 0.25) {
//            drive.shooterMotor.setPower(0.75);
//            ShooterSpinup = false;
//        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("currentT", mpc.currentT);
        panelsTelemetry.update(telemetry);
    }



    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.043, 119.473),

                                    new Pose(56.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 88.000),

                                    new Pose(55.000, 89.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(144))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.000, 89.000),
                                    new Pose(52.927, 83.651),
                                    new Pose(34.209, 84.135)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.209, 84.135),

                                    new Pose(14.684, 84.135)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.684, 84.135),
                                    new Pose(34.209, 84.458),
                                    new Pose(49.377, 93.816)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.377, 93.816),

                                    new Pose(58.252, 114.543)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))

                    .build();
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
            telemetry.addData("Path 4 time", "%.3f", path4EndTime - autoStartTime);
            telemetry.addData("Path 5 time", "%.3f", path5EndTime - path1EndTime);
            telemetry.addData("Path 6 time", "%.3f", path6EndTime - path2EndTime);
            telemetry.addData("Total time",  "%.3f", path6EndTime - autoStartTime);
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
                mpc.setActivePath(paths.Path1);
                setPathState(2);
                break;

            case 2:
                if (mpc.isPathComplete() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path1EndTime = runtime.seconds();
                    mpc.setActivePath(paths.Path2);
                    setPathState(3);
                }
                break;

            case 3:
                if (mpc.isPathComplete() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path2EndTime = runtime.seconds();
                    mpc.setActivePath(paths.Path3);
                    setPathState(4);
                }
                break;

            case 4:
                if (mpc.isPathComplete() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path3EndTime = runtime.seconds();
                    mpc.setActivePath(paths.Path4);
                    setPathState(5);
                }
                break;

            case 5:
                if (mpc.isPathComplete() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path4EndTime = runtime.seconds();
                    mpc.setActivePath(paths.Path5);
                    setPathState(6);
                }
                break;

            case 6:
                if (mpc.isPathComplete() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path5EndTime = runtime.seconds();
                    mpc.setActivePath(paths.Path6);
                    setPathState(7);
                }
                break;

            case 7:
                if (mpc.isPathComplete() && pathState != -1 && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    path6EndTime = runtime.seconds();
                    telemetry.addData("=== TIMING ===", "");
                    telemetry.addData("Path 1 time", "%.3f", path1EndTime - autoStartTime);
                    telemetry.addData("Path 2 time", "%.3f", path2EndTime - path1EndTime);
                    telemetry.addData("Path 3 time", "%.3f", path3EndTime - path2EndTime);
                    telemetry.addData("Path 4 time", "%.3f", path4EndTime - path3EndTime);
                    telemetry.addData("Path 5 time", "%.3f", path5EndTime - path4EndTime);
                    telemetry.addData("Path 6 time", "%.3f", path6EndTime - path5EndTime);
                    telemetry.addData("Total time",  "%.3f", path6EndTime - autoStartTime);
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