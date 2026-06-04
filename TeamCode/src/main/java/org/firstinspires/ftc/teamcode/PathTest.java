package org.firstinspires.ftc.teamcode;
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


@Autonomous(name = "Path Test", group = "Autonomous")
@Configurable // Panels
public class PathTest extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;
    public int pos = 0;
    public static double waitTime = 1;
    private char ball1 = 'P'; // to be changed
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
    private int seenAprilTag = -1;            // the confirmed tag ID we saw (or -1 if none)
    private int aprilTagConfirmCount = 0;    // how many consecutive frames we saw the same tag
    private static final int APRILTAG_CONFIRM_THRESHOLD = 3; // require N frames to confirm
    private final int[] targetTags = {1, 2, 3};
    public double EncoderZero;
    private LightweightMPC mpc;  //------ Testing

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        detectedBall3 = 'U';
        detectedBall2 = 'U';
        detectedBall1 = 'U';

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        mpc = new LightweightMPC(follower, drive, telemetry);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        drive.kicker.setPosition(0);
        pos = drive.intakeMotor.getCurrentPosition();
        kickerPos = drive.kicker.getPosition();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        colorScanner = new ColorSensingAuto(this, "Webcam 1");
//        motor = hardwareMap.get(DcMotor.class, "intake_motor");
        drive.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        drive.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        drive.light.setPosition(1);

        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
//        if (follower.isBusy()) {
//            mpc.update();
//        }
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        colorScanner.update();
        if (ShooterSpinup && follower.isBusy() && 0.25 <= follower.getCurrentTValue() && follower.getCurrentTValue() <= 1) {
            drive.shooterMotor.setPower(0.75);
            ShooterSpinup = false;
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(56.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 36.000),

                                    new Pose(95.725, 35.976)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    public int autonomousPathUpdate() {
        // Safety: if we're done, ensure everything stays stopped
        if (pathState == -1) {
            drive.shooterMotor.setPower(0);
            drive.spindexer.setPower(0);
            drive.intakeMotor.setPower(0);
            drive.frontLeftDrive.setPower(0);
            drive.backLeftDrive.setPower(0);
            drive.frontRightDrive.setPower(0);
            drive.backRightDrive.setPower(0);
            return -1;
        }

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:
                drive.frontLeftDrive.setPower(0);
                drive.backLeftDrive.setPower(0);
                drive.frontRightDrive.setPower(0);
                drive.backRightDrive.setPower(0);
                drive.intakeMotor.setPower(0);
                drive.shooterMotor.setPower(0);
                drive.spindexer.setPower(0);
                setPathState(1);
                break;
            case 1:
                telemetry.addData("case: ", 1);
                follower.followPath(paths.Path1, true);
                setPathState(2);
                break;
            case 2:
                telemetry.addData("case: ", 2);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathState != -1) {
                    telemetry.addLine("Successfully (or not) completed 3 ball auto");
                    telemetry.update();
                    drive.frontLeftDrive.setPower(0);
                    drive.backLeftDrive.setPower(0);
                    drive.frontRightDrive.setPower(0);
                    drive.backRightDrive.setPower(0);
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