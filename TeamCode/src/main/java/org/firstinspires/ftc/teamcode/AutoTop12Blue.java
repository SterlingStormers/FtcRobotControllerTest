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

@Autonomous(name = "AutoTop12Blue", group = "Autonomous")
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
    public boolean ShooterSpinup = false;
    public double EncoderZero;
    public boolean Spindexer1Special = false;




    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        double t = follower.getCurrentTValue();

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
        if (ShooterSpinup == true && follower.getCurrentTValue() >= 0.5 && follower.isBusy()) {
                drive.shooterMotor.setPower(1);
                ShooterSpinup = false;
        }
        SpindexerLogic1Special();
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
    public void SpindexerLogic1(){
        pos = drive.intakeMotor.getCurrentPosition();
        if (detectedBall3 == ball1) { // detectedBall3 is at slot2
            int remaining = 9557 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    slot2 = false;
                    setPathState(pathState + 1);
                }
            }

//--------------------------------USE FOR TIMER-----------------------------------
//                    if (Math.abs(remaining) <= 35) {
//                        power = 0;
//                    }
//                    if (Math.abs(remaining) >= 100) {
//                        pathTimer.resetTimer();
//                    }
//                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//
//                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {

        } else if (detectedBall2 == ball1) { // detectedBall2 is at slot1
            int remaining = pos - 6827; //cw
            double power = 0;
            power = (-0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);

            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);

            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    slot1 = false;
                    setPathState(pathState + 1);
                }
            }
        } else if (detectedBall1 == ball1) { //detectedBall1 is at slot0
            int remaining = 12288 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    has180Occured = true;
                    slot0 = false;
                    setPathState(pathState + 1);
                }
            }
        } else {
            telemetry.addLine("There has been an error with the amount of balls expected. Continuing auto");
            telemetry.update();
            setPathState(pathState + 1);
        }
    }
    public void SpindexerLogic1Special() {
        if (Spindexer1Special == true) {
            pos = drive.intakeMotor.getCurrentPosition();
            if (detectedBall3 == ball1) { // detectedBall3 is at slot2
                int remaining = 9557 - pos; //ccw
                double power = 0;
                power = (0.0005 * remaining);
                power = Math.max(power, -1);
                power = Math.min(power, 1);
                if (Math.abs(remaining) <= 35) {
                    power = 0;
                }
                if (Math.abs(remaining) >= 100) {
                    pathTimer.resetTimer();
                }
                telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                drive.spindexer.setPower(power);
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    drive.spindexer.setPower(0);
                    if (!kickerUp && !follower.isBusy()) {
                        drive.kicker.setPosition(kickerPos + 1);
                        telemetry.addData("kickerUp", true);
                        if (drive.kicker.getPosition() > kickerPos + 0.5) {
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                    }
                    if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                        drive.kicker.setPosition(kickerPos);
                        kickerUp = false;
                        slot2 = false;
                        Spindexer1Special = false;
                    }
                }

//--------------------------------USE FOR TIMER-----------------------------------
//                    if (Math.abs(remaining) <= 35) {
//                        power = 0;
//                    }
//                    if (Math.abs(remaining) >= 100) {
//                        pathTimer.resetTimer();
//                    }
//                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//
//                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {

            } else if (detectedBall2 == ball1) { // detectedBall2 is at slot1
                int remaining = pos - 6827; //cw
                double power = 0;
                power = (-0.0005 * remaining);
                power = Math.max(power, -1);
                power = Math.min(power, 1);

                if (Math.abs(remaining) <= 35) {
                    power = 0;
                }
                if (Math.abs(remaining) >= 100) {
                    pathTimer.resetTimer();
                }
                telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                drive.spindexer.setPower(power);

                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    drive.spindexer.setPower(0);
                    if (!kickerUp && !follower.isBusy()) {
                        drive.kicker.setPosition(kickerPos + 1);
                        telemetry.addData("kickerUp", true);
                        if (drive.kicker.getPosition() > kickerPos + 0.5) {
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                    }
                    if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                        drive.kicker.setPosition(kickerPos);
                        kickerUp = false;
                        slot1 = false;
                        Spindexer1Special = false;
                    }
                }
            } else if (detectedBall1 == ball1) { //detectedBall1 is at slot0
                int remaining = 12288 - pos; //ccw
                double power = 0;
                power = (0.0005 * remaining);
                power = Math.max(power, -1);
                power = Math.min(power, 1);
                if (Math.abs(remaining) <= 35) {
                    power = 0;
                }
                if (Math.abs(remaining) >= 100) {
                    pathTimer.resetTimer();
                }
                telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                drive.spindexer.setPower(power);
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    drive.spindexer.setPower(0);
                    if (!kickerUp && !follower.isBusy()) {
                        drive.kicker.setPosition(kickerPos + 1);
                        telemetry.addData("kickerUp", true);
                        if (drive.kicker.getPosition() > kickerPos + 0.5) {
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                    }
                    if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                        drive.kicker.setPosition(kickerPos);
                        kickerUp = false;
                        has180Occured = true;
                        slot0 = false;
                        Spindexer1Special = false;
                    }
                }
            } else {
                telemetry.addLine("There has been an error with the amount of balls expected. Continuing auto");
                telemetry.update();
                Spindexer1Special = false;
            }
        }
    }
    public void SpindexerLogic2() {
        pos = drive.intakeMotor.getCurrentPosition();
        if (detectedBall3 == ball2 && slot2 && has180Occured) { // detectedBall3 is at slot2
            int remaining = pos - 9557; //cw
            double power = 0;
            power = (-0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    has180Occured = false;
                    slot2 = false;
                    setPathState(pathState + 1);
                }
            }
        } else if (detectedBall3 == ball2 && slot2 && !has180Occured) { // detectedBall3 is at slot2
            int remaining = 9557 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    slot2 = false;
                    setPathState(pathState + 1); //15019
                }
            }
        } else if (detectedBall2 == ball2 && slot1 && has180Occured) { //detectedBall2 is at slot1
            int remaining = 15019 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    has180Occured = false;
                    slot1 = false;
                    setPathState(pathState + 1);
                }
            }
        } else if (detectedBall2 == ball2 && slot1 && !has180Occured) { //detectedBall2 is at slot1
            int remaining = 6827 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    slot1 = false;
                    setPathState(pathState + 1);
                }
            }
        } else if (detectedBall1 == ball2 && slot0) { //detectedBall1 is at slot0
            int remaining = 12288 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    has180Occured = true;
                    slot0 = false;
                    setPathState(pathState + 1);
                }
            }
        } else {
            telemetry.addLine("There has been an error with the amount of balls expected. Continuing auto");
            telemetry.update();
            setPathState(pathState + 1);
        }

    }
    public void SpindexerLogic3() {
        pos = drive.intakeMotor.getCurrentPosition();
        if (detectedBall3 == ball3 && slot2 && has180Occured) { // detectedBall3 is at slot2
            int remaining = pos - 9557; //cw
            double power = 0;
            power = (-0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    has180Occured = false;
                    slot2 = false;
                    setPathState(pathState + 1);
                }
            }
        } else if (detectedBall3 == ball3 && slot2 && !has180Occured) { // detectedBall3 is at slot2
            int remaining = 9557 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    slot2 = false;
                    setPathState(pathState + 1); //15019
                }
            }
        } else if (detectedBall2 == ball3 && slot1 && has180Occured) { //detectedBall2 is at slot1
            int remaining = 15019 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    has180Occured = false;
                    slot1 = false;
                    setPathState(pathState + 1);
                }
            }
        } else if (detectedBall2 == ball3 && slot1 && !has180Occured) { //detectedBall2 is at slot1
            int remaining = 6827 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    slot1 = false;
                    setPathState(pathState + 1);
                }
            }
        } else if (detectedBall1 == ball3 && slot0) { //detectedBall1 is at slot0
            int remaining = 12288 - pos; //ccw
            double power = 0;
            power = (0.0005 * remaining);
            power = Math.max(power, -1);
            power = Math.min(power, 1);
            if (Math.abs(remaining) <= 35) {
                power = 0;
            }
            if (Math.abs(remaining) >= 100) {
                pathTimer.resetTimer();
            }
            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
            drive.spindexer.setPower(power);
            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    telemetry.addData("kickerUp", true);
                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
                        kickerUp = true;
                        kickerStartTime = runtime.seconds();
                    }
                }
                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                    drive.kicker.setPosition(kickerPos);
                    kickerUp = false;
                    slot0 = false;
                    setPathState(pathState + 1);
                }
            }
        } else {
            telemetry.addLine("There has been an error with the amount of balls expected. Continuing auto");
            telemetry.update();
            setPathState(pathState + 1);
        }

    }
    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
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
                ShooterSpinup = true;
                setPathState(2);
                break;
            case 2:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot0 = true;
                        setPathState(3);
                    }
                    drive.spindexer.setPower(power);
                }
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
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos;
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot1 = true;
                        setPathState(5);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 5:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall2 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall2 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(6);
                }
                break;
            case 6:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 8192 - pos;
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot2 = true;
                        setPathState(7);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 7:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall3 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall3 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(8);
                }
                break;
            case 8:
                // husky lens
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(10);
                }
                break;
            case 10:
                SpindexerLogic1();
                break;
            case 11:
                SpindexerLogic2();
                break;
            case 12:
                SpindexerLogic3();
                break;
            case 13:
                drive.shooterMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, false);
                    setPathState(14);
                }
                break;
            case 14:
                if (follower.getCurrentTValue() >= 0.5 && follower.isBusy()) {
                    drive. intakeMotor.setPower(1);
                    setPathState(15);
                }
                break;
            case 15:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    int remaining = pos - 0; //ccw remember ccw if larger num before smaller num if pos is larger then switch them and ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        setPathState(16);
                    }

                    drive.spindexer.setPower(power);
                }

                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, false);
                    follower.setMaxPower(0.8);
                    setPathState(17);
                }
                break;
            case 17:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot0 = true;
                        setPathState(18);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 18:
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
                    setPathState(19);
                }
                break;
            case 19:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos;// ccw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot1 = true;
                        setPathState(20);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 20:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall2 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall2 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(21);
                }
                break;
            case 21:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                   int remaining = 8192 - pos;
                   double  power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot2 = true;
                        setPathState(22);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 22:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall3 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall3 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(23);
                }
                break;
            case 23:
                follower.setMaxPower(1);
                drive.intakeMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);

                    setPathState(24);
                }
                break;
            case 24:
                ShooterSpinup = true;
               Spindexer1Special = true;
               setPathState(25);
                break;
            case 25:
                //husky lens stuff
                setPathState(26);
                break;
            case 26:
                if (Spindexer1Special == false) {
                SpindexerLogic2();
                }
                break;
            case 27:
                SpindexerLogic3();
                break;
            case 28:
                drive.shooterMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, false);
                    setPathState(29);
                }
                break;
            case 29:
                if (follower.getCurrentTValue() >= 0.5 && follower.isBusy()) {
                    drive. intakeMotor.setPower(1);
                    setPathState(30);
                }
                break;
            case 30:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    int remaining = pos - 0; //ccw remember ccw if larger num before smaller num if pos is larger then switch them and ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        setPathState(31);
                    }

                    drive.spindexer.setPower(power);
                }

                break;
            case 31:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.Path7, false);
                    setPathState(32);
                }
                break;
            case 32:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot0 = true;
                        setPathState(33);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 33:
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
                    setPathState(34);
                }
                break;
            case 34:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos;// ccw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot1 = true;
                        setPathState(35);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 35:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall2 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall2 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(36);
                }
                break;
            case 36:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 8192 - pos;
                    double  power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot2 = true;
                        setPathState(37);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 37:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall3 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall3 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(38);
                }
                break;
            case 38:
                drive.intakeMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, false);
                    setPathState(39);
                }
                break;
            case 39:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(40);
                }
                break;
            case 40:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    if (pathTimer.getElapsedTimeSeconds() >= 2) {
                        follower.setMaxPower(1);
                        follower.followPath(paths.Path10, true);
                        setPathState(41);
                    }
                }
                break;
            case 41:
                Spindexer1Special = true;
                ShooterSpinup = true;
                setPathState(42);
                break;
            case 42:
                if (Spindexer1Special == false) {
                    SpindexerLogic2();
                }
                break;
            case 43:
                SpindexerLogic3();
                break;
            case 44:
                drive.shooterMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, false);
                    setPathState(45);
                }
                break;
            case 45:
                if (follower.getCurrentTValue() >= 0.5 && follower.isBusy()) {
                    drive. intakeMotor.setPower(1);
                    setPathState(46);
                }
                break;
            case 46:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    int remaining = pos - 0; //ccw remember ccw if larger num before smaller num if pos is larger then switch them and ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        setPathState(47);
                    }

                    drive.spindexer.setPower(power);
                }

                break;
            case 47:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.Path12, false);
                    setPathState(48);
                }
                break;
            case 48:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot0 = true;
                        setPathState(49);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 49:
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
                    setPathState(50);
                }
                break;
            case 50:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos;// ccw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot1 = true;
                        setPathState(51);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 51:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall2 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall2 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(52);
                }
                break;
            case 52:
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 8192 - pos;
                    double  power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot2 = true;
                        setPathState(53);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 53:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    if (colorScanner.detectedColor != null) {
                        detectedBall3 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                    } else {
                        detectedBall3 = 'U';
                    }
                    colorScanner.reset();
                    setPathState(54);
                }
                break;
            case 54:
                follower.setMaxPower(1);
                drive.intakeMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path13, true);
                    setPathState(55);
                }
                break;
            case 55:
                ShooterSpinup = true;
                Spindexer1Special = true;
                setPathState(56);
                break;
            case 56:
                if (Spindexer1Special == false) {
                    SpindexerLogic2();
                }
                break;
            case 57:
                SpindexerLogic3();
                break;
            case 58:
                drive.shooterMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path14, true);
                    setPathState(59);
                }
                break;
            case 59:
                if (!follower.isBusy() && pathState != -1) {
                    telemetry.addLine("Successfully (or not) completed 12 ball auto");
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
