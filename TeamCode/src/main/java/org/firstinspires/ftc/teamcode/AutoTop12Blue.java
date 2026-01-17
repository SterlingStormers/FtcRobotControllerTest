package org.firstinspires.ftc.teamcode;



import com.pedropathing.util.Timer;
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
import org.firstinspires.ftc.teamcode.ColorSensingAuto;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoTop12Blue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;
    public int pos = 0;
    public static double waitTime = 0.5;
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




    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
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
        drive.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Makes sure intake motor does not rely on

}

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        colorScanner.update();
        pos = drive.intakeMotor.getCurrentPosition();
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
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.875, 119.473),

                                    new Pose(72.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(72.000, 72.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(142))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(52.927, 83.651),
                                    new Pose(34.209, 84.135)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))

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
                            new BezierCurve(
                                    new Pose(49.377, 93.816),
                                    new Pose(55.024, 60.092),
                                    new Pose(34.854, 60.253)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.854, 60.253),

                                    new Pose(9.359, 59.931)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.359, 59.931),

                                    new Pose(28.722, 60.092)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(28.722, 60.092),
                                    new Pose(26.625, 71.064),
                                    new Pose(16.298, 70.258)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.298, 70.258),
                                    new Pose(44.697, 76.712),
                                    new Pose(49.538, 93.655)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.538, 93.655),
                                    new Pose(63.899, 38.469),
                                    new Pose(35.016, 36.210)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.016, 36.210),

                                    new Pose(9.036, 36.049)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.036, 36.049),
                                    new Pose(47.118, 51.217),
                                    new Pose(49.538, 93.978)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.538, 93.978),

                                    new Pose(58.897, 110.114)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))

                    .build();
        }
    }
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        double t = follower.getCurrentTValue();
        follower.setMaxPower(1);
        switch(pathState) {
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
                follower.followPath(paths.Path1, true);
                setPathState(2);
                break;
            case 2:
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot0 = true;
                        setPathState(3);
                    }
                drive.spindexer.setPower(power);
                break;
            case 3:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall1 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall1 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(4);
                }
            break;
            case 4:
                remaining = 5462 - pos;
                power = 0;
                power = (-0.0005 * remaining);
                power = Math.max(power, -1);
                power = Math.min(power, 1);
                if (Math.abs(remaining) <= 35) {
                    power = 0;
                    slot1 = true;
                    setPathState(5);
                }
                drive.spindexer.setPower(power);
                break;
        }















        return pathState;
    }
}
