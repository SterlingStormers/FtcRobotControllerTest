package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Spindexer Logic", group = "Autonomous")
@Configurable
public class SpindexerLogic extends OpMode {
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double waitTime = 0.5;
    private char ball1 = 'P';
    private char ball2 = 'G';
    private char ball3 = 'P';
    private char detectedBall1 = 'P';
    private char detectedBall2 = 'P';
    private char detectedBall3 = 'G';
    private boolean slot0 = false;
    private boolean slot1 = false;
    private boolean slot2 = false;
    private DcMotor motor;
    private boolean kickerUp = false;
    private double kickerPos = 0;
    private double kickerStartTime = 0.0;
    public boolean has180Occured = false;
    private ElapsedTime runtime = new ElapsedTime();
    public int pos = 0;

    // Track which slot is currently being shot
    private int activeSlot = -1;  // 0, 1, or 2

    @Override
    public void init() {
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        drive.kicker.setPosition(0);
        pos = drive.intakeMotor.getCurrentPosition();
        kickerPos = drive.kicker.getPosition();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        drive.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // debug telemetry for kicker timing
        telemetry.addData("pathState", pathState);
        telemetry.addData("kickerStartTime", kickerStartTime);
        telemetry.addData("kickerElapsed", kickerStartTime > 0.0 ? runtime.seconds() - kickerStartTime : 0.0);
        telemetry.addData("activeSlot", activeSlot);

        // If kicker has been up for >= 0.5s, lower it and advance the state
        if (kickerStartTime > 0.0 && (runtime.seconds() - kickerStartTime >= 0.5)) {
            // ensure kicker is down
            drive.kicker.setPosition(kickerPos);
            kickerUp = false;

            // Clear the correct slot based on which one was active
            if (activeSlot == 2) {
                slot2 = false;
            } else if (activeSlot == 1) {
                slot1 = false;
            } else if (activeSlot == 0) {
                slot0 = false;
                has180Occured = true;  // Only set when slot0 is shot
            }

            // Advance to next state
            if (pathState == 5) {
                setPathState(6);
            } else if (pathState == 6) {
                has180Occured = false;  // Reset for ball3
                setPathState(7);
            } else if (pathState == 7) {
                setPathState(8);
            }

            // Clear the active slot and timestamp
            activeSlot = -1;
            kickerStartTime = 0.0;
        }

        pathState = autonomousPathUpdate();
        telemetry.update();
    }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                drive.intakeMotor.setPower(-1);
                setPathState(1);
                telemetry.addData("case", 0);
                break;
            case 1:
                telemetry.addData("case", 1);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 2731 - pos;
                    double power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        slot0 = true;
                        setPathState(2);
                    } else {
                        drive.spindexer.setPower(power);
                    }
                }
                break;
            case 2:
                telemetry.addData("case", 2);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos;
                    double power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        slot1 = true;
                        setPathState(3);
                    } else {
                        drive.spindexer.setPower(power);
                    }
                }
                break;
            case 3:
                telemetry.addData("case", 3);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 8192 - pos;
                    double power = (0.0005 * Math.abs(remaining));
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        telemetry.addData("caseReached", 4);
                        drive.spindexer.setPower(0);
                        slot2 = true;
                        setPathState(4);
                    } else {
                        telemetry.addData("remaining: ", remaining);
                        drive.spindexer.setPower(power);
                    }
                }
                break;
            case 4:
                telemetry.addData("case", 4);
                drive.intakeMotor.setPower(0);
                drive.shooterMotor.setPower(1);
                setPathState(5);
                break;

            // ===== CASE 5 - Shoot ball1 =====
            case 5: {
                telemetry.addData("case", 5);
                pos = drive.intakeMotor.getCurrentPosition();

                if (detectedBall3 == ball1) {  // ball1 is in slot2
                    int remaining = 9557 - pos;
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 2;  // Track that we're shooting slot2
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }

                } else if (detectedBall2 == ball1) {  // ball1 is in slot1
                    int remaining = pos - 6827;
                    double power = -0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 1;  // Track that we're shooting slot1
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }

                } else if (detectedBall1 == ball1) {  // ball1 is in slot0
                    int remaining = 12288 - pos;
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 0;  // Track that we're shooting slot0
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }
                }
            }
            break;

            // ===== CASE 6 - Shoot ball2 =====
            case 6:
                telemetry.addData("case", 6);
                pos = drive.intakeMotor.getCurrentPosition();

                if (detectedBall3 == ball2 && slot2) {
                    int remaining = has180Occured ? (pos - 9557) : (9557 - pos);
                    double power = has180Occured ? (-0.0005 * remaining) : (0.0005 * remaining);
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 2;
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }

                } else if (detectedBall2 == ball2 && slot1) {
                    int remaining = has180Occured ? (15019 - pos) : (6827 - pos);
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 1;
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }

                } else if (detectedBall1 == ball2 && slot0) {
                    int remaining = 12288 - pos;
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 0;
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }
                }
                break;

            // ===== CASE 7 - Shoot ball3 =====
            case 7:
                telemetry.addData("case", 7);
                pos = drive.intakeMotor.getCurrentPosition();

                if (detectedBall3 == ball3 && slot2) {
                    int remaining = has180Occured ? (pos - 9557) : (9557 - pos);
                    double power = has180Occured ? (-0.0005 * remaining) : (0.0005 * remaining);
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 2;
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }

                } else if (detectedBall2 == ball3 && slot1) {
                    int remaining = has180Occured ? (15019 - pos) : (6827 - pos);
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 1;
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }

                } else if (detectedBall1 == ball3 && slot0) {
                    int remaining = 12288 - pos;
                    double power = 0.0005 * remaining;
                    power = Math.max(-1, Math.min(1, power));

                    if (Math.abs(remaining) <= 35) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                            activeSlot = 0;
                        }
                    } else {
                        drive.spindexer.setPower(power);
                    }
                }
                break;

            case 8:
                drive.shooterMotor.setPower(0);
                drive.spindexer.setPower(0);
                setPathState(-1);
                break;
        }
        return pathState;
    }
}