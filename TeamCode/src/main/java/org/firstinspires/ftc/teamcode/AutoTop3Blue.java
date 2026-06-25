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

@Autonomous(name = "Auto Top 3 Blue", group = "Autonomous")
@Configurable
public class AutoTop3Blue extends OpMode {
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

    // V2 MPC stack
    private AMPCV1 mpc;
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

        // V2 MPC stack
        mpc = new AMPCV1(follower);
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

        // Give Pinpoint IMU time to calibrate before play (robot stationary)
        try { Thread.sleep(500); } catch (InterruptedException ignored) {}
        follower.updatePose();

        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        // V2 MPC stack: pose refresh + plan + track + drive every loop
        follower.updatePose();
        mpc.update();
        controller.velocity();
        kinematics.drive();

        pathState = autonomousPathUpdate();
        colorScanner.update();

        // Shooter spinup: while a path is being driven AND we're past 25% through it
        if (ShooterSpinup && !mpc.isPathComplete() && mpc.currentT >= 0.25) {
            drive.shooterMotor.setPower(0.75);
            ShooterSpinup = false;
        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("currentT", mpc.currentT);
        panelsTelemetry.debug("desired V", "(" + mpc.desiredVx + ", " + mpc.desiredVy + ", " + mpc.desiredOmega + ")");
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

    public void SpindexerLogic1(){
        pos = drive.intakeMotor.getCurrentPosition();

        if (detectedBall3 == ball1) {
            int remaining = 9557 - pos;
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));
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

        } else if (detectedBall2 == ball1) {
            int remaining = pos - 6827;
            double power = -0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));
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

        } else if (detectedBall1 == ball1) {
            int remaining = 12288 - pos;
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));
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


        pos = drive.intakeMotor.getCurrentPosition();

        if (detectedBall3 == ball2 && slot2) {
            int remaining = has180Occured ? (pos - 9557) : (9557 - pos);
            double power = has180Occured ? (-0.0005 * remaining) : (0.0005 * remaining);
            power = Math.max(-1, Math.min(1, power));
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
                has180Occured = false;
                slot2 = false;
                setPathState(pathState + 1);
            }

        } else if (detectedBall2 == ball2 && slot1) {
            int remaining = has180Occured ? (15019 - pos) : (6827 - pos);
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));
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

        } else if (detectedBall1 == ball2 && slot0) {
            int remaining = 12288 - pos;
            double power = 0.0005 * remaining;
            power = Math.max(-1, Math.min(1, power));
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
            drive.intakeMotor.setPower(1);
            setPathState(pathState + 1);
        }
    }

    public void SpindexerLogic3() {

        pos = drive.intakeMotor.getCurrentPosition();

        if (detectedBall3 == ball3 && slot2) {
            int remaining = has180Occured ? (pos - 9557) : (9557 - pos);
            double power = has180Occured ? (-0.0005 * remaining) : (0.0005 * remaining);
            power = Math.max(-1, Math.min(1, power));
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
            setPathState(pathState + 1);
        }
    }


    public int autonomousPathUpdate() {
        if (pathState == -1) {
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
                mpc.setActivePath(paths.Path1);   // ← was follower.followPath(paths.Path1, true)
                ShooterSpinup = true;
                setPathState(2);
                break;

            case 2:

                HuskyLens.Block[] blocks = drive.husky.blocks();

                boolean foundTargetThisFrame = false;
                for (int i = 0; i < blocks.length; i++) {

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

                    if (seenAprilTag == 1) {
                        ball1 = 'P'; ball2 = 'P'; ball3 = 'G';
                    } else if (seenAprilTag == 2) {
                        ball1 = 'P'; ball2 = 'G'; ball3 = 'P';
                    } else if (seenAprilTag == 3) {
                        ball1 = 'G'; ball2 = 'P'; ball3 = 'P';
                    } else {
                        ball1 = 'P'; ball2 = 'G'; ball3 = 'P';
                    }
                }


                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    int remaining = 2731 - pos;
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));
                    if (Math.abs(remaining) > 10) {
                        power = Math.max(Math.abs(power), 0.1) * Math.signum(power);
                    }
                    if (Math.abs(remaining) <= 35 && pathTimer.getElapsedTimeSeconds() >= waitTime) {
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
                    colorScanner.startScan();

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
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos;
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));
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
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));
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
                setPathState(9);
                break;

            case 9:
                if (mpc.isPathComplete()) {    // ← was !follower.isBusy()
                    mpc.setActivePath(paths.Path2);   // ← was follower.followPath(paths.Path2, true)

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


                if (mpc.isPathComplete() && pathTimer.getElapsedTimeSeconds() >= waitTime) {    // ← was !follower.isBusy()
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
                setPathState(14);
                break;

            case 14:
                drive.shooterMotor.setPower(0);
                if (mpc.isPathComplete()) {    // ← was !follower.isBusy()
                    mpc.setActivePath(paths.Path3);    // ← was follower.followPath(paths.Path3, true)

                    pos = drive.intakeMotor.getCurrentPosition();
                    if (pathTimer.getElapsedTimeSeconds() >= waitTime/2) {
                        int remaining = 8129 - pos;
                        double power = 0.0005 * remaining;
                        power = Math.max(-1, Math.min(1, power));
                        int tolerance = 30;
                        drive.spindexer.setPower(power);

                        double timeoutSec = 0.85;
                        if (Math.abs(remaining) <= tolerance && pathTimer.getElapsedTimeSeconds() >= timeoutSec) {
                            drive.spindexer.setPower(0);
                            setPathState(15);
                        } else if (pathTimer.getElapsedTimeSeconds() >= timeoutSec) {
                            drive.spindexer.setPower(0);
                            setPathState(15);
                        }
                    }
                }
                break;

            case 15:
                if (mpc.isPathComplete() && pathState != -1) {    // ← was !follower.isBusy()
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