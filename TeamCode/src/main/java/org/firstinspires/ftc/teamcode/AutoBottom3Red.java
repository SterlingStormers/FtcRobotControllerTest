package org.firstinspires.ftc.teamcode;
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

@Autonomous(name = "Auto Bottom 3 Red", group = "Autonomous")
@Configurable // Panels
public class AutoBottom3Red extends OpMode {
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

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87, 8.670, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

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
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        colorScanner.update();
        if (ShooterSpinup && follower.isBusy() && 0.25 <= follower.getCurrentTValue() && follower.getCurrentTValue() <= 1) {
            drive.shooterMotor.setPower(0.82);
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
                                    new Pose(86, 8.670),

                                    new Pose(88.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 88.000),

                                    new Pose(89.000, 89.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(36))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.000, 89.000),

                                    new Pose(95.596, 119.987)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();
        }
    }
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();

        // Reset kicker state when entering shooting states
        if (newState == 10 || newState == 11 || newState == 12) {
            kickerUp = false;
        }
    }
    public void SpindexerLogic1(){

        telemetry.addData("detectedBall1", detectedBall1);
        telemetry.addData("detectedBall2", detectedBall2);
        telemetry.addData("detectedBall3", detectedBall3);
        telemetry.addData("slot0,slot1,slot2", "%b, %b, %b", slot0, slot1, slot2);
        telemetry.addData("pos", drive.intakeMotor.getCurrentPosition());
        telemetry.update();

        pos = drive.intakeMotor.getCurrentPosition();

        if (detectedBall3 == ball1) {  // ball1 is in slot2
            int remaining = 9557 - pos;
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                kickerUp = false;
                slot2 = false;
                setPathState(pathState + 1);
            }

        } else if (detectedBall2 == ball1) {  // ball1 is in slot1
            int remaining = pos - 6827;
            double power = -0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                kickerUp = false;
                slot1 = false;
                setPathState(pathState + 1);
            }

        } else if (detectedBall1 == ball1) {  // ball1 is in slot0
            int remaining = 12288 - pos;
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                kickerUp = false;
                has180Occured = true;
                slot0 = false;
                setPathState(pathState + 1);
            }
        } else {
            drive.spindexer.setPower(0);
            setPathState(pathState + 1);
        }
    }
    public void SpindexerLogic2() {

        telemetry.addData("detectedBall1", detectedBall1);
        telemetry.addData("detectedBall2", detectedBall2);
        telemetry.addData("detectedBall3", detectedBall3);
        telemetry.addData("slot0,slot1,slot2", "%b, %b, %b", slot0, slot1, slot2);
        telemetry.addData("pos", drive.intakeMotor.getCurrentPosition());
        telemetry.addData("kickerUp", kickerUp);
        telemetry.update();

        pos = drive.intakeMotor.getCurrentPosition();

        if (detectedBall3 == ball2 && slot2) {
            int remaining = has180Occured ? (pos - 9557) : (9557 - pos);
            double power = has180Occured ? (-0.0005 * remaining) : (0.0005 * remaining);
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            telemetry.addData("Logic2: targeting slot2", true);
            telemetry.addData("remaining", Math.abs(remaining));

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                kickerUp = false;
                has180Occured = false;
                slot2 = false;
                setPathState(pathState + 1);
            }

        } else if (detectedBall2 == ball2 && slot1) {
            int remaining = has180Occured ? (15019 - pos) : (6827 - pos);
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            telemetry.addData("Logic2: targeting slot1", true);
            telemetry.addData("remaining", Math.abs(remaining));

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                kickerUp = false;
                slot1 = false;
                setPathState(pathState + 1);
            }

        } else if (detectedBall1 == ball2 && slot0) {
            int remaining = 12288 - pos;
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            telemetry.addData("Logic2: targeting slot0", true);
            telemetry.addData("remaining", Math.abs(remaining));

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                kickerUp = false;
                has180Occured = true;
                slot0 = false;
                setPathState(pathState + 1);
            }
        } else {
            drive.spindexer.setPower(0);
            drive.intakeMotor.setPower(1);
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

        if (detectedBall3 == ball3 && slot2) {
            int remaining = has180Occured ? (pos - 9557) : (9557 - pos);
            double power = has180Occured ? (-0.0005 * remaining) : (0.0005 * remaining);
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                drive.spindexer.setPower(0);
                kickerUp = false;
                has180Occured = false;
                slot2 = false;
                setPathState(pathState + 1);
            }

        } else if (detectedBall2 == ball3 && slot1) {
            int remaining = has180Occured ? (15019 - pos) : (6827 - pos);
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                drive.spindexer.setPower(0);
                kickerUp = false;
                has180Occured = false;
                slot1 = false;
                setPathState(pathState + 1);
            }

        } else if (detectedBall1 == ball3 && slot0) {
            int remaining = 12288 - pos;
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));

            // Prevent stalling - use minimum power when far away
            if (Math.abs(remaining) > 10) {
                power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
            }

            if (Math.abs(remaining) <= 10) {
                drive.spindexer.setPower(0);
                if (!kickerUp) {
                    drive.kicker.setPosition(kickerPos + 1);
                    kickerUp = true;
                    kickerStartTime = runtime.seconds();
                }
            } else {
                drive.spindexer.setPower(power);
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                drive.kicker.setPosition(kickerPos);
                drive.spindexer.setPower(0);
                kickerUp = false;
                slot0 = false;
                setPathState(pathState + 1);
            }
        } else {
            drive.spindexer.setPower(0);
            drive.intakeMotor.setPower(1);
            setPathState(pathState + 1);
        }
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

                ShooterSpinup = true;
                setPathState(2);
                break;
            case 2:
                telemetry.addData("case: ", 2);

                HuskyLens.Block[] blocks = drive.husky.blocks();
                telemetry.addData("Block count", blocks.length);

                boolean foundTargetThisFrame = false;
                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].toString());

                    int id = blocks[i].id;
                    for (int t = 0; t < targetTags.length; t++) {
                        if (id == targetTags[t]) {
                            foundTargetThisFrame = true;

                            if (seenAprilTag == id) {
                                aprilTagConfirmCount++;
                            } else {
                                seenAprilTag = id;
                                aprilTagConfirmCount = 1;
                            }

                            panelsTelemetry.debug("AprilTag candidate", id + " (count=" + aprilTagConfirmCount + ")");
                            break;
                        }
                    }

                    if (foundTargetThisFrame) break;
                }

                if (!foundTargetThisFrame) {
                    aprilTagConfirmCount = 0;
                    seenAprilTag = -1;
                }

                if (aprilTagConfirmCount >= APRILTAG_CONFIRM_THRESHOLD) {
                    panelsTelemetry.debug("AprilTag confirmed", String.valueOf(seenAprilTag));
                    telemetry.addData("AprilTag confirmed", seenAprilTag);
                    telemetry.update();

                    if (seenAprilTag == 1) {
                        ball1 = 'P';
                        ball2 = 'P';
                        ball3 = 'G';
                    } else if (seenAprilTag == 2) {
                        ball1 = 'P';
                        ball2 = 'G';
                        ball3 = 'P';
                    } else if (seenAprilTag == 3) {
                        ball1 = 'G';
                        ball2 = 'P';
                        ball3 = 'P';
                    } else {
                        ball1 = 'P';
                        ball2 = 'G';
                        ball3 = 'P';
                        telemetry.addData("scanned", false);
                    }
                }

                telemetry.update();

                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) > 10) {
                        power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
                    }
                    if (Math.abs(remaining) <= 35 && pathTimer.getElapsedTimeSeconds() >= waitTime) {
                        power = 0;
                        slot0 = true;
                        setPathState(3);
                    }
                    telemetry.addData("remaining: ", remaining);
                    drive.spindexer.setPower(power);
                }
                break;
            case 3:
                telemetry.addData("case:", 3);
                telemetry.addData("colorScanner.scanning (before)", colorScanner.scanning);
                telemetry.addData("colorScanner.colorReady (before)", colorScanner.colorReady);
                telemetry.update();

                if (!colorScanner.scanning && !colorScanner.colorReady) {
                    colorScanner.startScan();
                    colorScanner.startScan();
                    telemetry.addData("action", "startScan called");
                    telemetry.addData("scanStartTime", System.currentTimeMillis());
                    telemetry.update();
                }
                if (colorScanner.colorReady) {
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall1 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                        } else {
                            detectedBall1 = ball1;
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

                    if (Math.abs(remaining) > 10) {
                        power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
                    }

                    if (Math.abs(remaining) <= 35 && pathTimer.getElapsedTimeSeconds() >= waitTime) {
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
                    if (Math.abs(remaining) > 10) {
                        power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
                    }
                    if (Math.abs(remaining) <= 35 && pathTimer.getElapsedTimeSeconds() >= waitTime) {
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
                    try {
                        if (colorScanner.detectedColor != null) {
                            detectedBall3 = ColorSensingAuto.toBallChar(colorScanner.detectedColor);
                            // Force to P if ball1 or ball2 was already G
                        } else {
                            detectedBall3 = ball3;
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

                    if (detectedBall1 == 'G') {
                        detectedBall2 = 'P';
                        detectedBall3 = 'P';
                    } else {
                        detectedBall1 = 'P';
                        if (detectedBall2 == 'G') {
                            detectedBall3 = 'P';
                        } else {
                            detectedBall2 = 'P';
                            detectedBall3 = 'G';
                        }
                    }
                    if (detectedBall1 == 'P' && detectedBall2 == 'P' && detectedBall3 == 'P')  {
                        detectedBall1 = 'G';
                    }

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

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= waitTime) {
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
                telemetry.addData("case", 13);
                telemetry.update();
                setPathState(14);
                break;
            case 14:
                drive.shooterMotor.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);

                    pos = drive.intakeMotor.getCurrentPosition();
                    if (pathTimer.getElapsedTimeSeconds() >= waitTime/2) {

                        int remaining = 8129 - pos;
                        double power = 0;
                        power = (0.0005 * remaining);
                        power = Math.max(power, -1);
                        power = Math.min(power, 1);

                        int tolerance = 30;

                        drive.spindexer.setPower(power);
                        telemetry.addData("remaining: ", remaining);

                        double timeoutSec = 0.85;
                        if (Math.abs(remaining) <= tolerance && pathTimer.getElapsedTimeSeconds() >= timeoutSec) {
                            drive.spindexer.setPower(0);
                            setPathState(15);
                        } else if (pathTimer.getElapsedTimeSeconds() >= timeoutSec) {
                            telemetry.addData("timed out: ", true);
                            drive.spindexer.setPower(0);
                            setPathState(15);
                        }

                    }
                }
                break;
            case 15:
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