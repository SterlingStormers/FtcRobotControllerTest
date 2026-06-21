package org.firstinspires.ftc.teamcode;
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

@Autonomous(name = "Path Test Pedro", group = "Autonomous")
@Configurable
public class PathTestPedro extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;
    public int pos = 0;
    public static double waitTime = 1;
    private char ball1 = 'P';
    private char ball2 = 'G';
    private char ball3 = 'P';
    private char detectedBall1;
    private char detectedBall2;
    private char detectedBall3;
    private boolean slot0 = false;
    private boolean slot1 = false;
    private boolean slot2 = false;
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
    private double autoEndTime = -1;

    // V2 MPC stack
//    private AMPC mpc;
//    private VelocityControllerV2 controller;
//    private MecanumKinematics kinematics;

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

        // V2 MPC stack
//        mpc = new AMPC(follower);
//        controller = new VelocityControllerV2(follower, mpc);
//        kinematics = new MecanumKinematics(drive, mpc, controller);

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

        // Give Pinpoint IMU time to calibrate before play (robot stationary)
        try { Thread.sleep(500); } catch (InterruptedException ignored) {}
        follower.updatePose();

        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        // V2 MPC stack: pose refresh + plan + track + drive every loop
        follower.update();
//        mpc.update();
//        controller.velocity();
//        kinematics.drive();

        pathState = autonomousPathUpdate();
        colorScanner.update();

        // Shooter spinup: while a path is being driven AND we're past 25% through it
        if (ShooterSpinup && !follower.isBusy() && follower.getCurrentTValue() >= 0.25) {
            drive.shooterMotor.setPower(0.75);
            ShooterSpinup = false;
        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("currentT", follower.getCurrentTValue());
//        panelsTelemetry.debug("desired V", "(" + mpc.desiredVx + ", " + mpc.desiredVy + ", " + mpc.desiredOmega + ")");
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.907, 119.235),
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
                            new BezierLine(
                                    new Pose(55.000, 89.000),
                                    new Pose(48.404, 119.987)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
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
            telemetry.addData("Total auto time", autoEndTime - autoStartTime);
            telemetry.update();
            drive.shooterMotor.setPower(0);
            drive.spindexer.setPower(0);
            drive.intakeMotor.setPower(0);
            // Drive motors zeroed via MPC (no active path → desiredV* = 0 → kinematics writes 0)
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
                telemetry.addData("case: ", 1);
                follower.followPath(paths.Path1);   // ← was follower.followPath(paths.Path1, true)
                setPathState(2);
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathState != -1) {    // ← was !follower.isBusy()
                    autoEndTime = runtime.seconds();
                    telemetry.addData("Total auto time", autoEndTime - autoStartTime);
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