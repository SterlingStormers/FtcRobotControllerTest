
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

@Autonomous(name = "Auto Top 6 Blue", group = "Autonomous")
@Configurable // Panels
public class AutoTop6Blue extends OpMode {
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
    public double EncoderZero;
    public boolean kickerReady = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23.739, 119.067, Math.toRadians(90)));

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
        if (ShooterSpinup && follower.isBusy() && 0.25 <= follower.getCurrentTValue() && follower.getCurrentTValue() <= 1) {
            drive.shooterMotor.setPower(1);
            ShooterSpinup = false;
        }
        telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());

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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.739, 119.067),

                                    new Pose(72.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),

                                    new Pose(66.203, 77.688)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(66.203, 77.688),

                                    new Pose(41.086, 83.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.086, 83.721),

                                    new Pose(35.302, 83.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.302, 83.721),

                                    new Pose(30.158, 83.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30.158, 83.721),

                                    new Pose(24.968, 83.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.968, 83.721),

                                    new Pose(13.529, 83.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.529, 83.721),

                                    new Pose(66.351, 77.845)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(66.351, 77.845),

                                    new Pose(48.404, 119.987)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(132))

                    .build();
        }
    }
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

//    private void commandKickerUp() {
//        double upPos = Math.min(kickerPos + 1.0, 1.0); // clamp to valid range
//        drive.kicker.setPosition(upPos);
//        // Immediately mark kicker as commanded up and start the timer:
//        kickerUp = true;
//        kickerStartTime = runtime.seconds();
//    }

    public void SpindexerLogic1(){

        telemetry.addData("detectedBall1", detectedBall1);
        telemetry.addData("detectedBall2", detectedBall2);
        telemetry.addData("detectedBall3", detectedBall3);
        telemetry.addData("slot0,slot1,slot2", "%b, %b, %b", slot0, slot1, slot2);
        telemetry.addData("pos", drive.intakeMotor.getCurrentPosition());
        telemetry.update();

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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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
    public void SpindexerLogic2() {

        telemetry.addData("detectedBall1", detectedBall1);
        telemetry.addData("detectedBall2", detectedBall2);
        telemetry.addData("detectedBall3", detectedBall3);
        telemetry.addData("slot0,slot1,slot2", "%b, %b, %b", slot0, slot1, slot2);
        telemetry.addData("pos", drive.intakeMotor.getCurrentPosition());
        telemetry.update();

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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
                    drive.kicker.setPosition(kickerPos); // go back to home
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

        telemetry.addData("detectedBall1", detectedBall1);
        telemetry.addData("detectedBall2", detectedBall2);
        telemetry.addData("detectedBall3", detectedBall3);
        telemetry.addData("slot0,slot1,slot2", "%b, %b, %b", slot0, slot1, slot2);
        telemetry.addData("pos", drive.intakeMotor.getCurrentPosition());
        telemetry.update();

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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
        } else if (detectedBall1 == ball3 && slot0 ) { //detectedBall1 is at slot0
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
            if (Math.abs(remaining) > 45) {
                power = Math.max(Math.abs(power), 0.15) * Math.signum(power);
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
//    public void SpindexerSafety() {
//        pos = drive.intakeMotor.getCurrentPosition();
//        if (detectedBall3 == 'U' && has180Occured) { // detectedBall3 is at slot2
//            int remaining = pos - 9557; //cw
//            double power = 0;
//            power = (-0.0005 * remaining);
//            power = Math.max(power, -1);
//            power = Math.min(power, 1);
//            if (Math.abs(remaining) <= 35) {
//                power = 0;
//            }
//            if (Math.abs(remaining) >= 100) {
//                pathTimer.resetTimer();
//            }
//            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//            drive.spindexer.setPower(power);
//            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
//                drive.spindexer.setPower(0);
//                if (!kickerUp) {
//                    drive.kicker.setPosition(kickerPos + 1);
//                    telemetry.addData("kickerUp", true);
//                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
//                        kickerUp = true;
//                        kickerStartTime = runtime.seconds();
//                    }
//                }
//                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
//                    drive.kicker.setPosition(kickerPos);
//                    kickerUp = false;
//                    has180Occured = false;
//                    detectedBall3 = 'N';
//                    slot2 = false;
//                }
//            }
//        }  if (detectedBall3 == 'U'&& !has180Occured) { // detectedBall3 is at slot2
//            int remaining = 9557 - pos; //ccw
//            double power = 0;
//            power = (0.0005 * remaining);
//            power = Math.max(power, -1);
//            power = Math.min(power, 1);
//            if (Math.abs(remaining) <= 35) {
//                power = 0;
//            }
//            if (Math.abs(remaining) >= 100) {
//                pathTimer.resetTimer();
//            }
//            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//            drive.spindexer.setPower(power);
//            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
//                drive.spindexer.setPower(0);
//                if (!kickerUp) {
//                    drive.kicker.setPosition(kickerPos + 1);
//                    telemetry.addData("kickerUp", true);
//                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
//                        kickerUp = true;
//                        kickerStartTime = runtime.seconds();
//                    }
//                }
//                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
//                    drive.kicker.setPosition(kickerPos);
//                    kickerUp = false;
//                    detectedBall3 = 'N';
//                    slot2 = false;
//                }
//            }
//        }  if (detectedBall2 == 'U' && has180Occured) { //detectedBall2 is at slot1
//            int remaining = 15019 - pos; //ccw
//            double power = 0;
//            power = (0.0005 * remaining);
//            power = Math.max(power, -1);
//            power = Math.min(power, 1);
//            if (Math.abs(remaining) <= 35) {
//                power = 0;
//            }
//            if (Math.abs(remaining) >= 100) {
//                pathTimer.resetTimer();
//            }
//            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//            drive.spindexer.setPower(power);
//            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
//                drive.spindexer.setPower(0);
//                if (!kickerUp) {
//                    drive.kicker.setPosition(kickerPos + 1);
//                    telemetry.addData("kickerUp", true);
//                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
//                        kickerUp = true;
//                        kickerStartTime = runtime.seconds();
//                    }
//                }
//                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
//                    drive.kicker.setPosition(kickerPos);
//                    kickerUp = false;
//                    has180Occured = false;
//                    detectedBall2 = 'N';
//                    slot1 = false;
//                }
//            }
//        } if (detectedBall2 == 'U' && !has180Occured) { //detectedBall2 is at slot1
//            int remaining = 6827 - pos; //ccw
//            double power = 0;
//            power = (0.0005 * remaining);
//            power = Math.max(power, -1);
//            power = Math.min(power, 1);
//            if (Math.abs(remaining) <= 35) {
//                power = 0;
//            }
//            if (Math.abs(remaining) >= 100) {
//                pathTimer.resetTimer();
//            }
//            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//            drive.spindexer.setPower(power);
//            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
//                drive.spindexer.setPower(0);
//                if (!kickerUp) {
//                    drive.kicker.setPosition(kickerPos + 1);
//                    telemetry.addData("kickerUp", true);
//                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
//                        kickerUp = true;
//                        kickerStartTime = runtime.seconds();
//                    }
//                }
//                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
//                    drive.kicker.setPosition(kickerPos);
//                    kickerUp = false;
//                    detectedBall2 = 'N';
//                    slot1 = false;
//                }
//            }
//        }  if (detectedBall1 == 'U') { //detectedBall1 is at slot0
//            int remaining = 12288 - pos; //ccw
//            double power = 0;
//            power = (0.0005 * remaining);
//            power = Math.max(power, -1);
//            power = Math.min(power, 1);
//            if (Math.abs(remaining) <= 35) {
//                power = 0;
//            }
//            if (Math.abs(remaining) >= 100) {
//                pathTimer.resetTimer();
//            }
//            telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//            drive.spindexer.setPower(power);
//            if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
//                drive.spindexer.setPower(0);
//                if (!kickerUp) {
//                    drive.kicker.setPosition(kickerPos + 1);
//                    telemetry.addData("kickerUp", true);
//                    if (drive.kicker.getPosition() > kickerPos + 0.5) {
//                        kickerUp = true;
//                        kickerStartTime = runtime.seconds();
//                    }
//                }
//                if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
//                    drive.kicker.setPosition(kickerPos);
//                    kickerUp = false;
//                    detectedBall1 = 'N';
//                    slot0 = false;
//                }
//            } if  ((detectedBall1 != 'U') || (detectedBall2 != 'U') || (detectedBall3 != 'U')){
//                setPathState(pathState + 1);
//            }
//        }
//    }
    public void SpindexerSpecial() {

        telemetry.addData("detectedBall1", detectedBall1);
        telemetry.addData("detectedBall2", detectedBall2);
        telemetry.addData("detectedBall3", detectedBall3);
        telemetry.addData("slot0,slot1,slot2", "%b, %b, %b", slot0, slot1, slot2);
        telemetry.addData("pos", drive.intakeMotor.getCurrentPosition());
        telemetry.update();

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
                slot2 = false;
                kickerReady = true;
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
                slot1 = false;
               kickerReady = true;
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
                slot0 = false;
                kickerReady = true;
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
                ShooterSpinup = true;
                setPathState(2);
                break;
            case 2:
                telemetry.addData("case: ", 2);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35 && pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        power = 0;
                        slot0 = true;
                        setPathState(3);
                    }
                    if (Math.abs(remaining) > 35) {
                        power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
                    }
                    telemetry.addData("remaining: ", remaining);
                    drive.spindexer.setPower(power);
                }
                break;
            case 3:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall1 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                        }
                    } catch (IllegalStateException e) {
                        detectedBall1 = ball1;
                    }
                    colorScanner.reset();
                    setPathState(4);
                }
                break;

            case 4:
                telemetry.addData("case: ", 4);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos;
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35 && pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        power = 0;
                        slot1 = true;
                        setPathState(5);
                    }
                    if (Math.abs(remaining) > 35) {
                        power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 5:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall2 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                        }
                    } catch (IllegalStateException e) {
                        detectedBall2 = ball2;
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
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35 && pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        power = 0;
                        slot2 = true;
                        setPathState(7);
                    }
                    if (Math.abs(remaining) > 35) {
                        power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 7:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall3 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                        }
                    } catch (IllegalStateException e) {
                        detectedBall3 = ball3;
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
                    telemetry.addData("followPath: ", 2);
                    follower.followPath(paths.Path2, true);
                    setPathState(10);
                }
                break;
            case 10:
                telemetry.addData("detectedBall1", detectedBall1);
                telemetry.addData("detectedBall2", detectedBall2);
                telemetry.addData("detectedBall3", detectedBall3);
                telemetry.addData("slot0,slot1,slot2", "%b, %b, %b", slot0, slot1, slot2);
                telemetry.addData("pos", drive.intakeMotor.getCurrentPosition());
                telemetry.update();

                if (!follower.isBusy()) {
                    SpindexerLogic1();
                }
                break;
            case 11:
                SpindexerLogic2();
                break;
            case 12:
                SpindexerLogic3();
                break;
            case 13:
                //SpindexerSafety();
                setPathState(14);
                break;
            case 14:
                drive.shooterMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, false);
                        setPathState(15);

                }
                break;
            case 15:
                drive.intakeMotor.setPower(-1);
                pos = drive.intakeMotor.getCurrentPosition();
                    int remaining = pos; //ccw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);

                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 75) {
                        pathTimer.resetTimer();
                    }

                    if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        drive.spindexer.setPower(0);
                        setPathState(16);
                    }

                    telemetry.addData("remaining: ", remaining);
                    drive.spindexer.setPower(power);
                break;
            case 16:
                //drive.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("position: ", drive.intakeMotor.getCurrentPosition());
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.45);
                    follower.followPath(paths.Path4, true);
                    if (pathTimer.getElapsedTimeSeconds() >= waitTime ) {
                        setPathState(17);
                    }
                }
                break;
            case 17:
                pos = drive.intakeMotor.getCurrentPosition();
                    remaining = 2731 - pos; //ccw
                    power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);

                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 75) {
                        pathTimer.resetTimer();
                    }

                    if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        slot0 = true;
                        drive.spindexer.setPower(0);
                        setPathState(18);
                    }

                    telemetry.addData("remaining: ", remaining);
                    drive.spindexer.setPower(power);
                break;
            case 18:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall1 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                        }
                    } catch (IllegalStateException e) {
                        detectedBall1 = ball1;
                    }
                    colorScanner.reset();
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5,true);
                    if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        setPathState(20);
                    }
                }
                break;
            case 20:
                pos = drive.intakeMotor.getCurrentPosition();
                    remaining = 5462 - pos; //ccw
                    power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);

                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }

                    if (Math.abs(remaining) >= 75) {
                        pathTimer.resetTimer();
                    }

                    if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        slot1 = true;
                        drive.spindexer.setPower(0);
                        setPathState(21);
                    }

                    telemetry.addData("remaining: ", remaining);
                    drive.spindexer.setPower(power);
                break;
            case 21:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall2 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                        }
                    } catch (IllegalStateException e) {
                        detectedBall2 = ball2;
                    }
                    colorScanner.reset();
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, false);
                    if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        setPathState(23);
                    }
                }
                break;
            case 23:
                pos = drive.intakeMotor.getCurrentPosition();
                    remaining = 8192 - pos;
                    power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);

                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }

                    if (Math.abs(remaining) >= 75) {
                        pathTimer.resetTimer();
                    }

                    if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        slot2 = true;
                        drive.spindexer.setPower(0);
                        setPathState(24);
                    }

                    drive.spindexer.setPower(power);
                break;
            case 24:
                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                }
                if (colorScanner.colorReady) {
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall3 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                        }
                    } catch (IllegalStateException e) {
                        detectedBall3 = ball3;
                    }
                    colorScanner.reset();
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7,false);
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path8, true);
                    ShooterSpinup = true;
                    setPathState(27);
                }
                break;
            case 27:
                SpindexerSpecial();
                if (!follower.isBusy() && kickerReady) {
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
                        setPathState(28);
                        }
                    }
                break;
            case 28:
                SpindexerLogic2();
                break;
            case 29:
                SpindexerLogic3();
                break;
            case 30:
                //SpindexerSafety();
                setPathState(31);
                break;
            case 31:
                follower.followPath(paths.Path9, true);
                setPathState(32);
                break;
            case 32:
                if (!follower.isBusy() && pathState != -1) {
                    telemetry.addLine("Successfully (or not) completed 6 ball auto");
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